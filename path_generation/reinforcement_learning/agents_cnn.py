"""
agents_cnn.py
PyTorch CNN 기반 RL 에이전트
- CNNFeatureExtractor: 공유 CNN 백본
- CNNPPOAgent: PPO (권장)
- CNNDQNAgent: DQN (대안)
"""

import random
from collections import deque
from typing import Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim


# ---------------------------------------------------------------------------
# Feature Extractor
# ---------------------------------------------------------------------------
class CNNFeatureExtractor(nn.Module):
    """
    CNN 백본 + aux 입력 concat → 128차원 특징 벡터
    Input:
      grid_obs: (batch, 7, 32, 32)
      aux:      (batch, 4)
    Output:
      features: (batch, 128)
    """

    def __init__(self, n_channels=8, fixed_size=32, aux_dim=4):
        super().__init__()

        self.conv_layers = nn.Sequential(
            # Conv1: (8, 32, 32) → (32, 16, 16)
            nn.Conv2d(n_channels, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(2),

            # Conv2: (32, 16, 16) → (64, 8, 8)
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2),

            # Conv3: (64, 8, 8) → (128, 4, 4)
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4)),
        )

        # Flatten → 128*4*4 = 2048
        self.fc1 = nn.Linear(2048 + aux_dim, 256)
        self.dropout = nn.Dropout(0.1)
        self.fc2 = nn.Linear(256, 128)

    def forward(self, grid_obs: torch.Tensor, aux: torch.Tensor) -> torch.Tensor:
        """
        grid_obs: (batch, 8, 32, 32)
        aux:      (batch, 4)
        returns:  (batch, 128)
        """
        x = self.conv_layers(grid_obs)    # (batch, 128, 4, 4)
        x = x.flatten(1)                  # (batch, 2048)
        x = torch.cat([x, aux], dim=1)    # (batch, 2052)
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        return x                           # (batch, 128)


# ---------------------------------------------------------------------------
# PPO Agent
# ---------------------------------------------------------------------------
class CNNPPOAgent:
    """
    PPO 알고리즘 (권장)
    """

    def __init__(
        self,
        n_channels=8,
        fixed_size=32,
        n_actions=4,
        aux_dim=4,
        lr=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_ratio=0.2,
        value_coef=0.5,
        entropy_coef=0.01,
        max_grad_norm=0.5,
        ppo_epochs=4,
        batch_size=64,
    ):
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_ratio = clip_ratio
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm
        self.ppo_epochs = ppo_epochs
        self.batch_size = batch_size
        self.n_actions = n_actions

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.feature_extractor = CNNFeatureExtractor(n_channels, fixed_size, aux_dim).to(self.device)
        self.policy_head = nn.Linear(128, n_actions).to(self.device)
        self.value_head = nn.Linear(128, 1).to(self.device)

        self.optimizer = optim.Adam(
            list(self.feature_extractor.parameters()) +
            list(self.policy_head.parameters()) +
            list(self.value_head.parameters()),
            lr=lr,
        )

        # 롤아웃 버퍼
        self.buffer_obs = []
        self.buffer_aux = []
        self.buffer_actions = []
        self.buffer_log_probs = []
        self.buffer_values = []
        self.buffer_rewards = []
        self.buffer_dones = []

    def act(self, obs: np.ndarray, aux: np.ndarray) -> Tuple[int, float, float]:
        """
        returns: (action, log_prob, value)
        """
        self.feature_extractor.eval()
        self.policy_head.eval()
        self.value_head.eval()

        with torch.no_grad():
            obs_t = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
            aux_t = torch.FloatTensor(aux).unsqueeze(0).to(self.device)

            features = self.feature_extractor(obs_t, aux_t)
            logits = self.policy_head(features)
            value = self.value_head(features).squeeze(-1)

            dist = torch.distributions.Categorical(logits=logits)
            action = dist.sample()
            log_prob = dist.log_prob(action)

        return action.item(), log_prob.item(), value.item()

    def store(self, obs, aux, action, log_prob, value, reward, done):
        self.buffer_obs.append(obs)
        self.buffer_aux.append(aux)
        self.buffer_actions.append(action)
        self.buffer_log_probs.append(log_prob)
        self.buffer_values.append(value)
        self.buffer_rewards.append(reward)
        self.buffer_dones.append(done)

    def update(self) -> Tuple[float, float, float]:
        """
        PPO 업데이트
        returns: (policy_loss, value_loss, entropy_loss)
        """
        if len(self.buffer_rewards) == 0:
            return 0.0, 0.0, 0.0

        self.feature_extractor.train()
        self.policy_head.train()
        self.value_head.train()

        # GAE 계산
        returns, advantages = self._compute_gae()

        # 텐서 변환
        obs_t = torch.FloatTensor(np.array(self.buffer_obs)).to(self.device)
        aux_t = torch.FloatTensor(np.array(self.buffer_aux)).to(self.device)
        actions_t = torch.LongTensor(self.buffer_actions).to(self.device)
        old_log_probs_t = torch.FloatTensor(self.buffer_log_probs).to(self.device)
        returns_t = torch.FloatTensor(returns).to(self.device)
        advantages_t = torch.FloatTensor(advantages).to(self.device)
        advantages_t = (advantages_t - advantages_t.mean()) / (advantages_t.std() + 1e-8)

        n = len(self.buffer_rewards)
        total_policy_loss = 0.0
        total_value_loss = 0.0
        total_entropy_loss = 0.0
        update_count = 0

        for _ in range(self.ppo_epochs):
            indices = np.random.permutation(n)
            for start in range(0, n, self.batch_size):
                idx = indices[start:start + self.batch_size]

                features = self.feature_extractor(obs_t[idx], aux_t[idx])
                logits = self.policy_head(features)
                values = self.value_head(features).squeeze(-1)

                dist = torch.distributions.Categorical(logits=logits)
                new_log_probs = dist.log_prob(actions_t[idx])
                entropy = dist.entropy().mean()

                ratio = torch.exp(new_log_probs - old_log_probs_t[idx])
                adv = advantages_t[idx]

                # PPO clipped objective
                surr1 = ratio * adv
                surr2 = torch.clamp(ratio, 1 - self.clip_ratio, 1 + self.clip_ratio) * adv
                policy_loss = -torch.min(surr1, surr2).mean()

                value_loss = F.mse_loss(values, returns_t[idx])
                entropy_loss = -entropy

                loss = (policy_loss
                        + self.value_coef * value_loss
                        + self.entropy_coef * entropy_loss)

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(
                    list(self.feature_extractor.parameters()) +
                    list(self.policy_head.parameters()) +
                    list(self.value_head.parameters()),
                    self.max_grad_norm,
                )
                self.optimizer.step()

                total_policy_loss += policy_loss.item()
                total_value_loss += value_loss.item()
                total_entropy_loss += entropy_loss.item()
                update_count += 1

        self._clear_buffer()

        if update_count == 0:
            return 0.0, 0.0, 0.0

        return (total_policy_loss / update_count,
                total_value_loss / update_count,
                total_entropy_loss / update_count)

    def _compute_gae(self):
        n = len(self.buffer_rewards)
        returns = np.zeros(n, dtype=np.float32)
        advantages = np.zeros(n, dtype=np.float32)
        last_gae = 0.0
        last_value = 0.0

        for t in reversed(range(n)):
            if self.buffer_dones[t]:
                last_value = 0.0
                last_gae = 0.0
            delta = (self.buffer_rewards[t]
                     + self.gamma * last_value
                     - self.buffer_values[t])
            last_gae = delta + self.gamma * self.gae_lambda * last_gae
            advantages[t] = last_gae
            returns[t] = last_gae + self.buffer_values[t]
            last_value = self.buffer_values[t]

        return returns, advantages

    def _clear_buffer(self):
        self.buffer_obs.clear()
        self.buffer_aux.clear()
        self.buffer_actions.clear()
        self.buffer_log_probs.clear()
        self.buffer_values.clear()
        self.buffer_rewards.clear()
        self.buffer_dones.clear()

    def save(self, path: str):
        torch.save({
            'feature_extractor': self.feature_extractor.state_dict(),
            'policy_head': self.policy_head.state_dict(),
            'value_head': self.value_head.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'config': {
                'fixed_size': 32,
                'n_channels': 7,
                'n_actions': 4,
                'aux_dim': 4,
            },
        }, path)

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.feature_extractor.load_state_dict(ckpt['feature_extractor'])
        self.policy_head.load_state_dict(ckpt['policy_head'])
        self.value_head.load_state_dict(ckpt['value_head'])
        if 'optimizer' in ckpt:
            self.optimizer.load_state_dict(ckpt['optimizer'])

    @torch.no_grad()
    def predict(self, obs: np.ndarray, aux: np.ndarray) -> int:
        """결정론적 추론 (greedy)"""
        self.feature_extractor.eval()
        self.policy_head.eval()
        obs_t = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
        aux_t = torch.FloatTensor(aux).unsqueeze(0).to(self.device)
        features = self.feature_extractor(obs_t, aux_t)
        logits = self.policy_head(features)
        return logits.argmax(dim=-1).item()


# ---------------------------------------------------------------------------
# DQN Agent (대안)
# ---------------------------------------------------------------------------
class ReplayBuffer:
    def __init__(self, capacity: int):
        self.buffer = deque(maxlen=capacity)

    def push(self, obs, aux, action, reward, next_obs, next_aux, done):
        self.buffer.append((obs, aux, action, reward, next_obs, next_aux, done))

    def sample(self, batch_size: int):
        batch = random.sample(self.buffer, batch_size)
        obs, aux, actions, rewards, next_obs, next_aux, dones = zip(*batch)
        return (np.array(obs), np.array(aux), np.array(actions),
                np.array(rewards, dtype=np.float32),
                np.array(next_obs), np.array(next_aux),
                np.array(dones, dtype=np.float32))

    def __len__(self):
        return len(self.buffer)


class CNNDQNAgent:
    """
    DQN 알고리즘 (대안)
    주의: 리플레이 버퍼 메모리 = 50000 × 7 × 32 × 32 × 4 bytes ≈ 1.4 GB
    메모리 부족 시 memory_size=20000 권장
    """

    def __init__(
        self,
        n_channels=8,
        fixed_size=32,
        n_actions=4,
        aux_dim=4,
        lr=1e-3,
        gamma=0.99,
        epsilon_start=1.0,
        epsilon_end=0.01,
        epsilon_decay=0.9995,
        memory_size=20000,
        batch_size=32,
        target_update=100,
    ):
        self.gamma = gamma
        self.epsilon = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.batch_size = batch_size
        self.target_update = target_update
        self.n_actions = n_actions
        self.step_count = 0

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.online_net = self._build_net(n_channels, fixed_size, n_actions, aux_dim)
        self.target_net = self._build_net(n_channels, fixed_size, n_actions, aux_dim)
        self.target_net.load_state_dict(self.online_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.online_net.parameters(), lr=lr)
        self.memory = ReplayBuffer(memory_size)

    def _build_net(self, n_channels, fixed_size, n_actions, aux_dim):
        extractor = CNNFeatureExtractor(n_channels, fixed_size, aux_dim).to(self.device)
        head = nn.Linear(128, n_actions).to(self.device)
        return nn.ModuleList([extractor, head])

    def _forward(self, net, obs_t, aux_t):
        features = net[0](obs_t, aux_t)
        return net[1](features)

    def act(self, obs: np.ndarray, aux: np.ndarray) -> int:
        if random.random() < self.epsilon:
            return random.randint(0, self.n_actions - 1)

        with torch.no_grad():
            obs_t = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
            aux_t = torch.FloatTensor(aux).unsqueeze(0).to(self.device)
            q_values = self._forward(self.online_net, obs_t, aux_t)
            return q_values.argmax(dim=-1).item()

    def store(self, obs, aux, action, reward, next_obs, next_aux, done):
        self.memory.push(obs, aux, action, reward, next_obs, next_aux, done)

    def update(self) -> float:
        if len(self.memory) < self.batch_size:
            return 0.0

        obs, aux, actions, rewards, next_obs, next_aux, dones = self.memory.sample(self.batch_size)

        obs_t = torch.FloatTensor(obs).to(self.device)
        aux_t = torch.FloatTensor(aux).to(self.device)
        actions_t = torch.LongTensor(actions).to(self.device)
        rewards_t = torch.FloatTensor(rewards).to(self.device)
        next_obs_t = torch.FloatTensor(next_obs).to(self.device)
        next_aux_t = torch.FloatTensor(next_aux).to(self.device)
        dones_t = torch.FloatTensor(dones).to(self.device)

        # 현재 Q값
        q_values = self._forward(self.online_net, obs_t, aux_t)
        q_values = q_values.gather(1, actions_t.unsqueeze(1)).squeeze(1)

        # 타깃 Q값 (Double DQN)
        with torch.no_grad():
            next_actions = self._forward(self.online_net, next_obs_t, next_aux_t).argmax(1)
            next_q = self._forward(self.target_net, next_obs_t, next_aux_t)
            next_q = next_q.gather(1, next_actions.unsqueeze(1)).squeeze(1)
            target_q = rewards_t + self.gamma * next_q * (1 - dones_t)

        loss = F.smooth_l1_loss(q_values, target_q)
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.online_net.parameters(), 10.0)
        self.optimizer.step()

        # epsilon decay
        self.epsilon = max(self.epsilon_end, self.epsilon * self.epsilon_decay)

        # target network 업데이트
        self.step_count += 1
        if self.step_count % self.target_update == 0:
            self.target_net.load_state_dict(self.online_net.state_dict())

        return loss.item()

    def save(self, path: str):
        torch.save({
            'online_net': self.online_net.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'epsilon': self.epsilon,
            'config': {
                'fixed_size': 32,
                'n_channels': 7,
                'n_actions': 4,
                'aux_dim': 4,
            },
        }, path)

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.online_net.load_state_dict(ckpt['online_net'])
        self.target_net.load_state_dict(ckpt['online_net'])
        if 'optimizer' in ckpt:
            self.optimizer.load_state_dict(ckpt['optimizer'])
        if 'epsilon' in ckpt:
            self.epsilon = ckpt['epsilon']

    @torch.no_grad()
    def predict(self, obs: np.ndarray, aux: np.ndarray) -> int:
        obs_t = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
        aux_t = torch.FloatTensor(aux).unsqueeze(0).to(self.device)
        q_values = self._forward(self.online_net, obs_t, aux_t)
        return q_values.argmax(dim=-1).item()
