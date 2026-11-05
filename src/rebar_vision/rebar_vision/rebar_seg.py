#!/usr/bin/env python3
"""철근/주행영역 세그멘테이션 (DeepLabV3+ MobileNetV2, 7클래스) — 패키지 공용 모듈.

ROS 노드에서 쓰려고 `tools/vision_test/deeplabv3_seg_test.py`에 있던 추론부를
패키지로 이관한 것. tools 스크립트는 설치 경로에 안 들어가 노드가 import할 수 없다.
튜닝/분석 도구(tools/drive/*)는 반대로 이 모듈을 import한다.

⚠ 주행규칙: 로봇 무한궤도는 **철근 배근 위를 밟고** 주행 → 지지면=철근,
   background(방수포)·floor·wall·obstacle·human 전부 주행불가. ([[deck_edge]] 참조)

전제: pip install segmentation-models-pytorch
      (numpy 1.26.4 / opencv 4.10.0.84 핀 깨뜨리지 말 것)
"""
import numpy as np
import cv2
import torch

# ★★ [2026-09-14] **클래스맵을 체크포인트에서 자동 선택한다.**
#
# ## 왜 (사고 직전에 잡은 것)
# 9클래스 모델(rebar_seg_260914)이 나오면서 **기존 인덱스가 전부 밀렸다**:
#     idx 4: rebar_h → obstacle        idx 5: rebar_v → plate
#     idx 6: wall    → rebar_h         idx 3: obstacle → level_rod
# 그런데 `deck_edge.py` 는 `REBAR = [4, 5]` 처럼 **인덱스를 하드코딩**하고 있었다.
# 가중치만 바꾸면 "주행가능 = obstacle + plate", "주행불가 = 진짜 철근" 이 되어
# **판정이 통째로 뒤집힌다** — 데크 밖으로 나가는 그 사고 경로다.
# 지금은 head 크기 불일치로 로드가 실패해 시끄럽게 죽지만, 누군가 CLASS_MAP만
# 늘리고 deck_edge를 안 고치면 **조용히** 뒤집힌다.
#   ⟹ 인덱스는 **이름에서 유도**한다. 여기서 맵을 고르고, deck_edge.set_class_map()이
#      REBAR/NONDECK/OBSTACLE 을 이름으로 재계산한다. 클래스가 또 늘어도 안전하다.
# ⚠ **새 모델을 추가하면 여기에 맵을 등록할 것.** 등록 안 된 클래스 수는 로드가 거부된다
#    (추측해서 돌리는 것보다 안 도는 편이 낫다).
CLASS_MAPS = {
    7: {0: 'background', 1: 'floor', 2: 'human', 3: 'obstacle',
        4: 'rebar_h', 5: 'rebar_v', 6: 'wall'},
    # 2026-09-14 추가분: level_rod(노란 수직봉 — 전엔 색 기반으로만 잡았다),
    #                    plate(데크 가장자리 H빔 형상 철판 — **주행불가**)
    9: {0: 'background', 1: 'floor', 2: 'human', 3: 'level_rod', 4: 'obstacle',
        5: 'plate', 6: 'rebar_h', 7: 'rebar_v', 8: 'wall'},
}
CLASS_MAP = CLASS_MAPS[7]          # 하위호환 기본값(구 모델). 실제 값은 인스턴스가 갖는다

# BGR 팔레트 — **이름 기준**이라 인덱스가 밀려도 색이 안 바뀐다.
PALETTE_BY_NAME = {
    'background': (0, 0, 0), 'floor': (0, 140, 255), 'human': (0, 0, 255),
    'obstacle': (0, 0, 160), 'rebar_h': (0, 200, 0), 'rebar_v': (0, 200, 120),
    'wall': (150, 150, 150),
    'level_rod': (0, 255, 255),     # 노랑 — 실물 색과 맞춰 눈으로 바로 구분되게
    'plate': (200, 0, 200),         # 자홍 — 배근(초록)과 확실히 갈리게
}


def palette_for(class_map):
    return {i: PALETTE_BY_NAME.get(n, (255, 255, 255)) for i, n in class_map.items()}


PALETTE = palette_for(CLASS_MAP)
IDX = {v: k for k, v in CLASS_MAP.items()}          # 이름→인덱스 (구 모델 기준)


def classes_in_checkpoint(path):
    """체크포인트의 최종 conv out_channels = 클래스 수."""
    ck = torch.load(path, map_location='cpu', weights_only=False)
    sd = ck.get('state_dict', ck)
    for k in reversed(list(sd.keys())):
        if k.endswith('weight') and hasattr(sd[k], 'dim') and sd[k].dim() == 4:
            return int(sd[k].shape[0]), sd
    raise RuntimeError(f'{path}: 최종 conv 를 못 찾음 — 형식이 다른 체크포인트다')
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], np.float32)

DEFAULT_WEIGHTS = 'retrain_best_260804.pt'         # src/rebar_vision/model/ 기준


class RebarSegmenter:
    """SMP DeepLabV3+ 로드 + 추론."""

    def __init__(self, weights, device='cpu', imgsz=512, encoder='mobilenet_v2'):
        import segmentation_models_pytorch as smp
        self.device = device
        self.imgsz = imgsz
        # ★ 클래스 수를 **체크포인트에서 읽어** 넷을 만든다. 고정값으로 만들면
        #   모델이 바뀌었을 때 head 크기 불일치로 죽거나(그나마 다행), 같은 크기면
        #   조용히 다른 의미의 인덱스를 뱉는다. 위 CLASS_MAPS 주석 참조.
        n, sd = classes_in_checkpoint(weights)
        if n not in CLASS_MAPS:
            raise RuntimeError(
                f'{weights}: {n}클래스 모델인데 CLASS_MAPS 에 등록돼 있지 않다. '
                f'등록된 것: {sorted(CLASS_MAPS)}. '
                f'rebar_seg.py 의 CLASS_MAPS 에 이름 순서를 추가할 것 — '
                f'추측해서 돌리면 주행판정이 통째로 뒤집힌다.')
        self.class_map = CLASS_MAPS[n]
        self.idx = {v: k for k, v in self.class_map.items()}
        self.palette = palette_for(self.class_map)
        self.net = smp.DeepLabV3Plus(encoder_name=encoder, encoder_weights=None,
                                     in_channels=3, classes=n)
        # Lightning 'model.' 접두어 제거
        sd = {k[len('model.'):] if k.startswith('model.') else k: v for k, v in sd.items()}
        missing, unexpected = self.net.load_state_dict(sd, strict=False)
        if missing or unexpected:
            print(f'  [load] missing={len(missing)} unexpected={len(unexpected)} '
                  f'(0/0 이어야 정상)')
        self.net.eval().to(device)

    @torch.no_grad()
    def predict(self, bgr):
        """BGR 원본 → 원해상도 클래스 마스크(HxW uint8)."""
        h0, w0 = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(rgb, (self.imgsz, self.imgsz)).astype(np.float32) / 255.0
        img = (img - IMAGENET_MEAN) / IMAGENET_STD
        x = torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0).float().to(self.device)
        logits = self.net(x)                          # 1 x 7 x H x W
        m = logits.argmax(1)[0].to('cpu').numpy().astype(np.uint8)
        return cv2.resize(m, (w0, h0), interpolation=cv2.INTER_NEAREST)


def colorize(mask, class_map=None):
    """클래스 마스크 → BGR 오버레이 이미지.

    ⚠ `class_map` 을 넘기면 그 맵의 팔레트를 쓴다. 안 넘기면 구 7클래스 기준이라
       9클래스 마스크를 그리면 **색이 밀린다**(철근이 회색으로 보이는 식).
       호출부가 segmenter 를 갖고 있으면 `seg.class_map` 을 넘길 것.
    """
    pal = palette_for(class_map) if class_map else PALETTE
    out = np.zeros((*mask.shape, 3), np.uint8)
    for i, c in pal.items():
        out[mask == i] = c
    return out
