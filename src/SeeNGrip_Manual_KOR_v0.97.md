# SeeNGrip

![](images/f3ff024da0293de3ae67093dbb2b004111f78e08cc2dd852949210ebb8b2e65b.jpg)

# Optimum Series User Manual (Korean) Version 0.97

# 2024. Aug 3

Original Notice © 2024 SeeNGrip Inc.

# Table of Contents

# Revisions....

1. General.... 6

1.1 Specification....

1.2 Gripper Nomenclature..... 8

1.2.1 Optimum R - Nomenclature and Structure.. . 8   
1.2.2 Optimum A - Nomenclature and Structure.. 9   
1.2.3 Optimum P - Nomenclature and Structure.. ...... 10

# 1.3 Gripper Dimensions and Working Range..... .11

1.3.1 Optimum R - Dimensions and Working Range.   
1.3.2 Optimum A - Dimensions and Working Range.......... .......   
1.3.3 Optimum P - Dimensions and Working Range.. ..... 14

# 1.4 Force, Moment Limit.. . 15

# 1.5 Performance Charts.. 16

1.5.1. Optimum R - Performance Chart..... 17   
1.5.2. Optimum A - Performance Chart. . 19   
1.5.3. Optimum P - Performance Chart. ..... .21

# 2. Safety..... .23

3. Installation...... 24

3.1 Flange Assembly.. 24

3.2 Attachment Dimensions... . 24

3.2.1 Optimum R - Attachment Dimensions.. 25   
3.2.2 Optimum A - Attachment Dimensions.. 26   
3.2.3 Optimum P - Attachment Dimensions.. 27

# 3.3 Cable Connection.. .28

3.3.1 Pinouts for Universal Robot Tool Connector.... 28   
3.3.2 Pinouts of Rainbow Robotics Tool Connector.. 29   
3.3.3 Pinouts for Doosan Robotics Tool Connector.. .29

# 4. LED Indications..... . 30

# 5. Communication..... .. 31

5.1 Modbus RTU.... .. 31

5.2 Register Mapping....... . 33

5.2.1 Limit of Acceleration (Address 5, Configuration). 35   
5.2.2 Grip Torque (Address 6, Command Parameter)... .35   
5.2.3 Motion Torque (Address 7, Command Parameter).. .36   
5.2.4 Limit of Speed (Address 8, Command Parameter).. 36   
5.2.5 Grip Command (Address 9, Command). 38   
5.2.6 Fault Clear (Address 16, Configuration).. 44   
5.2.7 Release Retry Time (Address 17, Configuration).. 44   
5.2.8 Release Retry Range (Address 18, Configuration).. 44   
5.2.9 Grip Drive Time (Address 19, Configuration).. 45   
5.2.10 Grip Drive Range (Address 20, Configuration).. .46   
5.2.11 Home Retry Count (Address 21, Configuration).. .46   
5.2.12 Detect Speed (Address 22, Configuration).. .47   
5.3.13 Flash Write (Address 22, Configuration)... 47   
5.3.14 Comm Error Check (Address 62, Configuration).. .48   
5.3.15 Gripper Information (Address 2, Status).. .49   
5.3.16 Present Position (Address 10, Status).. .... 49   
  
5.3.17 Present Speed (Address 11, Status). 49   
5.3.18 Grip Position Deviation (Address 12, Status) 50   
5.3.19 Present Temperature (Address 13, Status). 50   
5.3.20 Present Power Voltage (Address 14, Status). 51   
5.3.21 Present Gripper Status (Address 15, Status). 52   
5.3.22 Present Fault (Address 16, Status).. 53   
5.3.23 Flash Write Count (Address 23, Status). .54   
5.3.24 Time Duration for Current Measurement during Speed Control   
(Address 64, Configuration). 54   
5.3.25 Average Current during Speed Control (Address 65,   
Configuration).. .... ....... 54

6. How to Use... . 55

6.1 Grasp Parameters and Procedures... 55

6.1.1 Move.. .. ... ..56   
6.1.2 Grip.. .57   
6.1.3 Speedy Grip.. ： 58

6.2 Grasp Applications (Optimum R, A)....... .. 59

7. Trouble Shooting... .. 62

7.1 Manual Release for Stuck Finger..... .. 62

7.2 Adjust Preload of Finger...... ....64

# Revisions

Revision 2024/1/2V0.9 작성 (최종 Map 반영, 성능 추가)  
Revision 2024/1/21V0.92 개정 (명칭 개선)  
Revision 2024/1/22V0.93 개정 (FW 2 버전, 통신 CRC, CheckSum, NoCheck 설정 추가)  
Revision 2024/1/30V0.94 개정 (Limit of Speed, Motion Torque, Grip Torque 초기화 오류 수정)  
Revision 2024/2/22V0.941 개정 (Present Gripper Status Command 내용 추가)  
Revision 2024/6/8V0.95 개정 (Grip, Speedy Grip 속도 한계 상향)  
Revision 2024/8/3V0.97 개정 (사양 업데이트)

# Copyright

$\circledcirc$ 2024 SeeNGrip Inc. All rights reserved.

This manual and the product it describes are protected by the Copyright Act of Korea, by laws of other countries, and by international treaties, and therefore may not be reproduced in whole or in part, whether for sale or not, without prior written consent from SeeNGrip. Under copyright law, copying includes translation into another language or format.

Information provided by SeeNGrip in this document is believed to be accurate and reliable. However, no responsibility is assumed by SeeNGrip for its use. There may be some difference between the manual and the product if the product has been modified after the edition date.

The information contained in this document is subject to change without notice

# 1. General

전동식 그리퍼 (Motorized Gripper 또는 Electric Gripper)는 공압식 그리퍼(PneumaticGripper 또는 Mechanical Gripper)에 비하여 아래의 장점이 있습니다.

1. 에너지 효율이 좋습니다.  
2. 부가 설비 (컴프레서, 레귤레이터, 필터, 윤활 장치, 각종 밸브)가 필요  
없습니다.  
3. 작동 시 소음과 진동이 적습니다.  
4. 수동 조정이 필요 없으며 디지털 통신으로 미세 조정이 가능합니다.

씬그립 Optimum Series 그리퍼는 로봇 및 자동화 장비에서 사용되는 프리미엄 전동그리퍼로써 다음의 특징들을 가지고 있다.

1. 중량(부피) 대비 속도 및 토크 최대화  
2. 드라이버 및 제어기 내장형  
3. 파지 위치/속도/힘 디지털 설정  
4. 전원 인가시 위치 확인 가능핑거측 절대 엔코더 내장  
5. 전원 차단시 파지 상태 유지웜 기어에 의한 역구동 방지  
6. 끼임 사고 방지 구조기구적 링크 구조가 외부로 드러나지 않음 (A 모델)  
7. 정밀한 파지 구조내압 베어링을 사용하여 백래쉬 최소화  
8. 핑거 사이 공간을 활용한 3점 파지 가능 (R, A 모델)  
9. Modbus RTU 표준 통신 방식  
10. ISO 9409 표준 로봇 취부 방식

# 1.1 Specification

Optimum Series는 로봇 응용에 따라 R, A, P 모델 중에 선택 가능합니다.

Optimum R는 원형 대상물 파지에 적합합니다. 예를 들어, 컵을 파지하는 경우에스프레소 잔부터 벤티 까지 다양한 컵을 하나의 그리퍼로 파지 가능하다는 장점이있습니다.

Optimum A는 파지 범위가 넓으면서 파지면이 항상 수평을 이루기 때문에 안정적파지가 가능합니다. 예를 들어, 머신 텐딩 응용과 같이 하나의 로봇으로 여러 종류의박스형 대상물을 파지해야 하는 경우 하나의 그리퍼로 여러 대상물을 파지할 수있다는 장점이 있습니다.

Optimum P는 항상 파지면이 수평을 이루기 때문에 안정적이 파지가 가능하면서,상대적으로 파지력이 크다는 장점이 있습니다. 예를 들어, 조리 도구를 파지해야 하는경우 높은 파지력으로 안정적인 파지가 가능하다는 장점이 있습니다.

Table 1.1: Specification of Optimum Series Grippers   

<table><tr><td rowspan=1 colspan=1>Model</td><td rowspan=1 colspan=1>Optimum R1</td><td rowspan=1 colspan=1>Optimum A</td><td rowspan=1 colspan=1>Optimum P</td></tr><tr><td rowspan=1 colspan=1>Image</td><td rowspan=1 colspan=1>1日1®0V  0</td><td rowspan=1 colspan=2>?000D    00000000     0000000®®O  O                    0                    O。O                             </td></tr><tr><td rowspan=1 colspan=1>Characteristics </td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1> Stroke</td><td rowspan=1 colspan=1>45 deg x 2</td><td rowspan=1 colspan=1>35 mm x 2</td><td rowspan=1 colspan=1>13 mm x 2</td></tr><tr><td rowspan=1 colspan=1>Max. S.Speed</td><td rowspan=1 colspan=1>280 deg/s x 2</td><td rowspan=1 colspan=1>250 mm/s x 2</td><td rowspan=1 colspan=1>50 mm/s x 2</td></tr><tr><td rowspan=1 colspan=1>Max. Torque (Force)</td><td rowspan=1 colspan=1>1.5 Nm</td><td rowspan=1 colspan=1>40 N</td><td rowspan=1 colspan=1>90 N</td></tr><tr><td rowspan=1 colspan=1>Weight</td><td rowspan=1 colspan=1>320g</td><td rowspan=1 colspan=1>445g</td><td rowspan=1 colspan=1>405g</td></tr><tr><td rowspan=1 colspan=1>Voltage</td><td rowspan=1 colspan=3>24 Vdc</td></tr><tr><td rowspan=1 colspan=1>Max. Current</td><td rowspan=1 colspan=3>1A</td></tr></table>

# 1.2 Gripper Nomenclature

# 1.2.1 Optimum R - Nomenclature and Structure

Optimum R은 부피와 자기 중량이 작고 최대 속도가 크면서, 경쟁력있는 토크를발생시키는 구조입니다. Finger와 Palm 사이는 Preload Bearing으로 연결되어 있어움직이는 회전 방향 이외에는 Backlash가 없습니다.

회전하는 Finger가 Palm 측면에 있기 때문에 필요시 3점 지지 파지 형태로도 사용할수 있으며, 대상물과 Flange 사이가 짧기 때문에 로봇 시스템의 정밀도를 높이는효과도 있습니다.

![](images/ef71352972ad0dd386d3b259d3b7047be30c229e6cfdcbb5857e329c3690bc1d.jpg)  
Fig. 1.1: SeeNGrip Optimum R

# 1.2.2 Optimum A - Nomenclature and Structure

Optimum A는 Finger Ends는 항상 수평을 이루기 때문에 안정적인 파지가 가능하며,회전형 Finger 구조로 작동 범위가 넓다는 장점이 있습니다.

Finger의 링크가 외부로 드러나지 않아서 링크 사이 끼임 사고가 발생 불가능한구조입니다. Finger와 Palm 사이는 Preload Bearing으로 연결되어 있어움직이는 회전 방향 이외에는 Backlash가 없습니다.

아래 그림에서와 같이 Optimum A는 Grasp된 경우 Finger Ends사이에 공간이발생되며 이 공간에 맞춤형 핑거를 제작할 경우 Flange와 대상물과의 거리를 최소화할 수 있습니다.

Finger가 Palm 측면에서 회전하는 구조로 Finger사이에 공간이 있어서 3점 지지형파지도 가능합니다.

![](images/358067f8ca32c16604250445420d59c4a7c199053dfc51f73cd0706677201c88.jpg)  
Fig. 1.2: SeeNGrip Optimum A

# 1.2.3 Optimum P - Nomenclature and Structure

Optimum P는 Optimum Series 내에서는 가장 큰 파지력을 가지고 있으며, 경쟁 제품대비 넓은 이동범위를 가집니다.

Finger는 Palm과 Linear Guide으로 연결되어 있어 Backlash가 적습니다.

![](images/b5bd13ee2e6f3a5b23ddababf5b68b14a1622da398e13d9d279c14cc59a4b531.jpg)  
Fig. 1.3: SeeNGrip Optimum P

# 1.3 Gripper Dimensions and Working Range

# Infomation

표기된 치수는 별도의 명기가 없는 경우 [mm] 단위이며, 그리퍼의 기준 좌표계는도면상에 붉은색 좌표계로 표시되어 있습니다.

# 1.3.1 Optimum R - Dimensions and Working Range

Optimum R은 넓은 파지 범위를 가지는 회전형 그리퍼로 Finger Preload AdjustHoles을 중심축으로 Finger가 Y축 회전하는 구조입니다.

Flange 기준 파지 위치는 Finger Tip 디자인에 따라 달라지기 때문에 여기서는 이동범위와 Finger Tip 설계에 필요한 치수를 표시합니다.

Optimum R의 이동 각도 범위는 최소 0 [deg]에서 최대 45 [deg] 입니다.

![](images/494aeacb4f65f8f54c3bd2044cbe3319e2c4d7520ccc4901d25ef63de106a607.jpg)  
Fig. 1.4: Dimensions and Working Range of Optimum R

# 1.3.2 Optimum A - Dimensions and Working Range

Optimum A는 회전하는 구조이지만, Finger에서 파지되는 부분은 항상 평행을이루게 되어 파지 안전성이 높은 구조입니다.

Optimum A의 Finger 사이의 거리는 Finger Tip이 없는 경우 13 [mm] 에서 83.7 [mm]이동이 가능하다. Optimum A의 Finger 사이 이동 범위는 좌우 $7 0 . 7 \mathsf { m m O }$ 며, Finger가움직임에 따라 상하로 14.6mm 이동이 발생됩니다.

![](images/3d9eef2713d196715fa0718cdae50334e8fb964d1e96a1df595be9592c330170.jpg)  
Fig. 1.5: Dimensions and Working Range of Optimum A

Optimum Series는 공통적으로 Grip Percentage로 조정되므로, Optimum A의 경우지령에 따른 Finger 사이 거리와 상하 높이 변화는 비례적이지 않으며 아래의방법으로 구할 수 있습니다.

Grip Percentage (GP): 0\~100[%] Gripping Angle (GA): 0\~45 [deg] Maximum Gripping Angle (MGA): 45 [deg] Initial Vertical Offset from Flange (IVO): 107.5 [mm]

Horizontal Distance $\mathrm { ( H D ) } = \mathsf { I H O } + 2 ^ { \star } \mathsf { L L } ^ { \star } \mathsf { s i n } \left( \mathsf { M G A } ^ { \star } \mathsf { G P } \right) [ \mathsf { m m } ]$ Height from Flange (HF) $=$ IVO - LL \* ( 1 - cos ( MGA \* GP) ) [mm]

HD는 Finger End 사이의 $\mathsf X$ 방향 거리를 나타내고, HF는 GP가 $0 \%$ 일때 GrippingReference Point (GRP)의 위치를 가지다가 GP 값이 증가함에 따라 Z 방향으로감소되는 값을 가집니다. GRP는 Finger End에 고정된 거리에 있는 값으로 Finger Tip설계에 따라 다른 기준을 설정할 수 있습니다. 본 문서에서는 GRP의 Z축 위치가GP가 $0 \%$ 경우 Initial Vertical Offset from Flange 값을 가지는 것으로 봅니다.

GP에 따른 GA, HD, HF의 변화는 아래 그래프와 같습니다.

![](images/45d0de09509d5257c29ad9f49689a34c253aacca402868faacb3243c590abcb6.jpg)  
Fig. 1.6: Operation Chart of Optimum A

# 1.3.3 Optimum P - Dimensions and Working Range

Optimum P의 는 X축 방향으로 Finger가 좌우로 이동하는 형태이며, 항상 파지접촉이 평행을 이루게 되어 안정적인 파지가 가능한 구조입니다.

Optimum P의 이동 범위는 최소 0 [mm] 에서 최대 26[mm] 입니다.Flange 기준 파지 위치는 Finger Tip 디자인에 따라 달라지기 때문에 여기서는 이동범위와 Finger Tip 설계에 필요한 치수를 표시합니다.

![](images/04110dd06e666639755f9a4aa95cb7ed58083e28d95a9c94a0434274f7fd8a27.jpg)

![](images/7649017fbd0f3ad2e39dcf594ff54b0cf7a964ba3f1def699a4e57aafa2c4ba7.jpg)  
Fig. 1.7: Gripping Range of Optimum P

# 1.4 Force, Moment Limit

Optimum Series는 최적화된 디자인으로 낮은 자중을 가지면서도 3Kg 또는 5Kg 급로봇용 그리퍼로 활용 가능합니다. 아래의 힘, 모멘트 한계 내에서 사용하여야 안전한사용이 가능합니다.

<table><tr><td rowspan=1 colspan=2>Model</td><td rowspan=1 colspan=1>Optimum R</td><td rowspan=1 colspan=1>Optimum A</td><td rowspan=1 colspan=1>Optimum P</td></tr><tr><td rowspan=3 colspan=1>Force[N]</td><td rowspan=1 colspan=1>X</td><td rowspan=1 colspan=1>1025</td><td rowspan=1 colspan=1>1025</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>Y</td><td rowspan=1 colspan=1>205</td><td rowspan=1 colspan=1>205        ！</td><td rowspan=1 colspan=1> 980</td></tr><tr><td rowspan=1 colspan=1>Z</td><td rowspan=1 colspan=1>205</td><td rowspan=1 colspan=1>205</td><td rowspan=1 colspan=1>980</td></tr><tr><td rowspan=3 colspan=1>Moment[Nm]</td><td rowspan=1 colspan=1>×</td><td rowspan=1 colspan=1>2.9</td><td rowspan=1 colspan=1>2.9</td><td rowspan=1 colspan=1>   4.7   </td></tr><tr><td rowspan=1 colspan=1>Y</td><td rowspan=1 colspan=1>2.9</td><td rowspan=1 colspan=1>1 2.9       </td><td rowspan=1 colspan=1>2.84</td></tr><tr><td rowspan=1 colspan=1>Z</td><td rowspan=1 colspan=1>-</td><td rowspan=1 colspan=1>15.3</td><td rowspan=1 colspan=1>2.84</td></tr></table>

Table 1.2: Force, moment Limit of Optimum Series

፠ 단일 핑거 기준, XYZ 방향은 그리퍼 Flange의 좌표계 방향፠ 본 값은 그리퍼의 요소 부품을 기반으로 계산된 사양입니다.

# 1.5 Performance Charts

성능 차트는 각 그리퍼별 Limit of Speed 및 Grip Torque에 따른 파지력, 속도에 따른파지력 및 위치 재현성, 파지력 유지 능력을 보여줍니다.

파지력 측정은 움직일 수 있는 대상물에 한쪽 핑거에서 20Kg 범위의 $1 \%$ 반복도를가지는 힘센서를 이용하여 계측되었습니다.

Limit of Speed 및 Grip Torque에 따른 파지력에서는 일반적으로 Grip Torque가 $0 \%$ 일경우 Limit of Speed에 비례하는 파지력이 관찰됩니다. Limit of Speed가 낮을경우에도 높은 파지력을 발생 가능하기 위해 Grip Torque를 설정하면 높은 파지력달성이 가능합니다.

속도에 따른 파지력 및 위치 재현성은 Grip Torque가 $0 \%$ 일 경우의 파지력을반복하였을때 오차의 크기를 최대 토크 사양의 백분율로 표현한다. 100회 측정한표준편차를 나타냅니다.

파지력 유지 능력은 낮은 Limit of Speed에서 Grip Torque로 파지력이 결정되었을때시간의 지남에 따라 감쇄하는 파지력 특성을 보여줍니다.

# Caution

옵티멈 그리퍼는 별도의 힘 센서가 없으므로 파지력은 환경 조건(온도, 습도, 윤활상황, 핑거의 구조 등)에 따라 파지력이 차이가 날 수 있다.

본 성능 자료의 목적은 옵티멈 그리퍼 시리즈의 기본 정보를 제공하는데 있습니다.측정 결과는 본 시험에서 사용한 매개변수 뿐만 아니라 , 사용자의 응용 상황 및조건에 따라 달라질 수 있으므로, 응용 및 환경 조건에 따라 별도의 계측을 통해핑거 설계 및 그리퍼 매개변수 선정이 필요할 수 있습니다.

# 1.5.1. Optimum R - Performance Chart

‘Grip’ 또는 ‘Speedy Grip’ 명령은 대상물을 감지하면 멈춘 후, 주어진 힘(GripTorque)을 가해주는 구조입니다. 주어진 힘의 크기가 0일 경우에는 속도(Limit ofSpeed)에 비례한 파지력 특성을 가지고 있으며, 속도가 $20 \%$ 낮은 경우에는 주어진힘에 따라 파지력을 제어할 수 있습니다.

Optimum R 그리퍼는 ‘Grip’ 또는 ‘Speedy Grip’에서의 $100 \%$ 속도는 위치 이동명령(Move)력에서의 최대 속도의 $50 \%$ 속도로 설정되어 있다. 이는 과도한 속도에의해 발생되는 관성에 대한 힘을 제한하기 위함입니다.

본 시험을 위해 사용한 Motion Torque는 30% 입니다. 대상물을 감지하여 멈출때까지 Motion Torque는 핑거를 이동하는데 사용할 수 있는 최대 토크입니다.

하기 값은 핑거의 회전 축 기준으로 50mm에서 측정한 하중입니다.

![](images/9a5d7e912ef334ccc72caef8753db1dafc812f90c2d4816d3faeeed326a5cc11.jpg)  
Fig. 1.8.1: Optimum R - Grip Force according to Limit of Speed and Grip Force

하기 좌측 그래프는 Grip Torque가 $0 \%$ 일 지령에서 속도를 달리해서 100회 반복해서측정하였을 경우에 관찰되는 평균 파지력과 파지력 오차의 표준 편차의 크기를

나타냅니다. 파지력 오차의 크기는 사양상의 최대 파지력에 대한 백분률로표현하였습니다.

하기 우측 그래프는 동일 시험에서 그리퍼 엔코더에서 관찰된 위치에 대한 평균값과위치 오차의 표준 편차의 크기를 나타냅니다.

![](images/39ff6f27da4b23082594a6fe6496eda6880530d583430500a76e081324ac6beb.jpg)  
본 측정은 Motion Torque가 $50 \%$ 일 때의 계측 결과입니다.  
Fig. 1.8.2: Optimum R - Repeatability of Grip Force and Position

하기 그래프는 속도에 따른 파지력이 결정되었을 경우 파지력이 얼마나유지되는지를 보여줍니다.

본 시험은 Limit of Speed가 20%이고, Motion Torque가 $50 \%$ 일 경우, Grip Torque에따라 외부센서에 의해 파지력을 100초 동안의 변화를 측정한 결과입니다.

![](images/86c85fab3743ed067ec0b26771dae72394d9163724d59cbfbb21608fd3a89446.jpg)  
Fig. 1.8.3: Optimum R - Grip Retention Capability

# 1.5.2. Optimum A - Performance Chart

‘Grip’ 또는 ‘Speedy Grip’ 명령은 대상물을 감지하면 멈춘 후, 주어진 힘(GripTorque)을 가해주는 구조입니다. 주어진 힘의 크기가 0일 경우에는 속도(Limit ofSpeed)에 비례한 파지력 특성을 가지고 있으며, 속도가 $20 \%$ 낮은 경우에는 주어진힘에 따라 파지력을 제어할 수 있습니다.

Optimum A 그리퍼는 ‘Grip’ 또는 ‘Speedy Grip’에서의 $100 \%$ 속도는 위치 이동명령(Move)력에서의 최대 속도의 $60 \%$ 속도로 설정되어 있다. 이는 과도한 속도에의해 발생되는 관성에 대한 힘을 제한하기 위함입니다.

본 시험을 위해 사용한 Motion Torque는 30% 입니다. 대상물을 감지하여 멈출때까지 Motion Torque는 핑거를 이동하는데 사용할 수 있는 최대 토크입니다.

![](images/e07d4ddaa81227fccacbbdd378c4585dc5aa53be98840a6e6182883416279e2a.jpg)  
Fig. 1.9.1: Optimum A - Grip Force according to Limit of Speed and Grip Force

하기 좌측 그래프는 Grip Torque가 $0 \%$ 일 지령에서 속도를 달리해서 100회 반복해서측정하였을 경우에 관찰되는 평균 파지력과 파지력 오차의 표준 편차의 크기를나타냅니다. 파지력 오차의 크기는 사양상의 최대 파지력에 대한 백분률로표현하였습니다.

하기 우측 그래프는 동일 시험에서 그리퍼 엔코더에서 관찰된 위치에 대한 평균값과위치 오차의 표준 편차의 크기를 나타냅니다.

![](images/d3d3aada91f04c4dd605181dcf9fca0aa62557d4e9a1ded21e1cac6a545e8140.jpg)  
본 측정은 Motion Torque가 $50 \%$ 일 때의 계측 결과입니다.  
Fig. 1.9.2: Optimum A - Repeatability of Grip Force and Position

하기 그래프는 속도에 따른 파지력이 결정되었을 경우 파지력이 얼마나유지되는지를 보여줍니다.

본 시험은 Limit of Speed가 $20 \% 0 1$ 고, Motion Torque가 50%일 경우, Grip Torque에따라 외부센서에 의해 파지력을 100초 동안의 변화를 측정한 결과입니다.

![](images/df69a4417465587cff13f2dd6b6a66e9816b6c63ecabf31c10407f72f2cee85b.jpg)

![](images/c72607fdda1d752a49687f21c26c22c7d6a8f9b6948b4f9c40066ccffd845e78.jpg)  
Fig. 1.9.3: Optimum A - Grip Retention Capability

# 1.5.3. Optimum P - Performance Chart

‘Grip’ 또는 ‘Speedy Grip’ 명령은 대상물을 감지하면 멈춘 후, 주어진 힘(GripTorque)을 가해주는 구조입니다. 주어진 힘의 크기가 0일 경우에는 속도(Limit ofSpeed)에 비례한 파지력 특성을 가지고 있으며, 속도가 $20 \%$ 낮은 경우에는 주어진힘에 따라 파지력을 제어할 수 있습니다. 옵티멈 $\mathsf { P } \equiv$ Limit of Speed에 따른 파지력변화보다 Grip Torque에 따른 파지력 변화가 크다는 특징이 있습니다.

Optimum P 그리퍼는 ‘Grip’ 또는 ‘Speedy Grip’에서의 $100 \%$ 속도는 위치 이동명령(Move)력에서의 최대 속도의 $50 \%$ 속도로 설정되어 있다. 이는 과도한 속도에의해 발생되는 관성에 대한 힘을 제한하기 위함입니다.

본 시험을 위해 사용한 Motion Torque는 $30 \%$ 입니다. 대상물을 감지하여 멈출때까지 Motion Torque는 핑거를 이동하는데 사용할 수 있는 최대 토크입니다.

![](images/c1d56875485f75060d46a8a4ae36aac081c1738c564e836646c89b564f2a3007.jpg)  
Fig. 1.10.1: Optimum P -Grip Force according to Limit of Speed and Grip Force

하기 좌측 그래프는 Grip Torque가 $0 \%$ 일 지령에서 속도를 달리해서 100회 반복해서측정하였을 경우에 관찰되는 평균 파지력과 파지력 오차의 표준 편차의 크기를

나타냅니다. 파지력 오차의 크기는 사양상의 최대 파지력에 대한 백분률로표현하였습니다.

하기 우측 그래프는 동일 시험에서 그리퍼 엔코더에서 관찰된 위치에 대한 평균값과위치 오차의 표준 편차의 크기를 나타냅니다.

![](images/92ab898b564b2f5793c76f47b4af4395ee627af993c5ce9cb61e5ba105024644.jpg)  
본 측정은 Motion Torque가 $20 \%$ 일 때의 계측 결과입니다.

![](images/cf0b2afc01d42ab54f9fff50bc9a7fed87c75cdb5ec8fa484a17c64a6c796e4b.jpg)  
Fig. 1.10.2: Optimum P - Repeatability of Grip Force and Position

하기 그래프는 속도에 따른 파지력이 결정되었을 경우 파지력이 얼마나유지되는지를 보여줍니다.

본 시험은 Limit of Speed가 $20 \% 0 1$ 고, Motion Torque가 $50 \%$ 일 경우, Grip Torque에따라 외부센서에 의해 파지력을 100초 동안의 변화를 측정한 결과입니다.

![](images/3b57f2bc3e8e200c85f43f79ae02c434bef8180428b6343bd1c8d66817f8d521.jpg)  
Fig. 1.10.3: Optimum P - Grip Retention Capability

# 2. Safety

# Caution

Any use of the gripper in noncompliance of these warnings is inappropriate and may cause injury or damage.

# Warning

The gripper needs to be properly secured before operating.   
Do not install or operate a damaged gripper   
Must supply voltage in the specification   
Appropriate flange bolting and cable connection should be checked before using a gripper   
Do not operate over range of force and moment limits.   
Do not use the gripper on people or animals.   
Do not place human finger between gripper fingers

# 3. Installation

# 3.1 Flange Assembly

Optimum Series는 로봇 및 자동화 장비와 그리퍼의 기계적인 연결은 ISO9409-1-50-4-M6 표준 방식을 따릅니다. 다른 방식의 Flange가 필요하신 경우제조사(www.seengrip.com)의 연락처를 통해 연락 부탁드립니다.

# 3.2 Attachment Dimensions

Optimum Series 그리퍼는 로봇 응용을 위해 Finger Tip, Hand Eye Camera, PalmTool 장착 가능합니다.

파지 목적에 따라 Finger Tip은 Finger 또는 Finger End에 장착하여 사용 가능합니다.  
Optimum Series는 로봇 응용을 위한 Finger Tip 디자인까지는 포함하지 않습니다.  
Finger Tip 장착을 의한 그리퍼의 치수 정보를 제공 드립니다.

Hand Eye Camera는 초점 조절 가능한 USB 연결 방식을 가지는 SeeNGrip Inc. 사의그리퍼용 카메라 제품으로 그리퍼에 Camera 부착 위치에 장착됩니다. 사용자는Hand Eye Camera 외에 다른 목적으로 해당 장착부를 사용할 수 있도록 CameraAttach 치수를 제공합니다. Camera 장착부는 Flange에 총 2개가 있으며, 그리퍼X축을 기준으로 상하부에 대칭되는 위치에 위치하고 있습니다. 필요에 따라 단안카메라 장착 위치 선정이 가능하고, 필요시 양쪽 모두 카메라를 장착하여 양안 카메라응용에도 활용 가능합니다.

Optimum Series는 완전 파지되었을 경우 (Grip Percentage가 0일 경우)에도 Finger사이에 공간이 있습니다. 이 공간을 활용하여 원하는 경우 3점 지지 방식으로대상물을 파지 할 수 있습니다 이때 별도의 Tool 또는 지그를 부탁하기 위한 Palm Tool장착부를 제공합니다.

# 3.2.1 Optimum R - Attachment Dimensions

![](images/b8f37a4f519aad550068c7b56ef6ee7b28c848f71a129b8829797dd49da2488f.jpg)  
Fig. 3.1: Attachments of Optimum P for Cameras and a Palm Tool

Optimum R의 Finger마다 2개의 M3 홀이 X 축 방향으로 장착하는 형태로 되어있습니다. 관통된 나사산이므로 안쪽에서 바깥쪽으로 또는 바깥쪽에서 안쪽으로볼트를 체결할 수 있습니다.

![](images/7b0082ffb91475a0f7d244a92f636dc609230e9b422f6a9bcec17ee88ddcd4a0.jpg)  
Fig. 3.2: Attachments of Optimum P for Finger Tips

# 3.2.2 Optimum A - Attachment Dimensions

![](images/0df9b91e16e8e5a5b4086e8f7df9bbe4d1eb3f8a40620a7709ef7b27347c0f45.jpg)  
Fig. 3.3: Attachments of Optimum A for Cameras and a Palm Tool

Optimum P의 Finger End는 3면에 Finger Tip을 장착하도록 설계되어 있어 다양한위치 또는 다수의 위치에 Finger Tip 장착이 가능한 장점을 가지고 있습니다. 또한방향마다 Dowel Pin을 위한 총 6개의 구멍이 있어서 보다 안정적인 Finger Tip이 장착가능합니다.

![](images/ba32b079ccbd13b6dbba1095cd5b75c852b938d8b0d7baddc9dcbcac7a975c97.jpg)  
Fig. 3.4: Attachments of Optimum P for Finger Tips

# 3.2.3 Optimum P - Attachment Dimensions

![](images/97941faaf44872021e9ca5a2aa838c7a2229ac1f9b3543f23aa7ba9a8c200563.jpg)  
Fig. 3.5: Attachments of Optimum A for Cameras and a Palm Tool

Optimum P의 Finger는 Y축 방향 또는 Z 축 방향으로 Finger Tip 장착 가능하며,Dowel Pin 장착을 위해 4개의 구멍이 있어 보다 안정적인 Finger Tip 장착이가능합니다.

![](images/f650b43b7a40fd88dfa5ee5bae9842bc8fbeb2272cdfa4d4023005b843f11b1e.jpg)  
Fig. 3.6: Attachments of Optimum P for Finger Tips

# 3.3 Cable Connection

Optimum Series 그리퍼는 로봇 팔의 툴 커넥터를 활용하여 연결할 수 있습니다. 로봇팔 제조사에 따라 Optimum Series에서 지원하는 케이블 종류는 아래와 같습니다.제조사 마다의 로봇팔 툴커넥터의 사양은 로봇팔 제조사 메뉴얼을 참고하시기바랍니다.

<table><tr><td rowspan=1 colspan=1>Robot Arm</td><td rowspan=1 colspan=1>Wrist Connection</td><td rowspan=1 colspan=1>Gripper</td></tr><tr><td rowspan=1 colspan=1>Universal Robot</td><td rowspan=1 colspan=1>M8-8pin-male</td><td rowspan=1 colspan=1> M8-8pin-female</td></tr><tr><td rowspan=1 colspan=1>Rainbow Robotics</td><td rowspan=1 colspan=1>M10-12pin-female ?</td><td rowspan=1 colspan=1> M10-12pin-male</td></tr><tr><td rowspan=1 colspan=1>Doosan Robotics</td><td rowspan=1 colspan=1>M8-8pin-female</td><td rowspan=1 colspan=1>M8-8pin-male</td></tr></table>

Table 3.1: Compatible Connector Types for Optimum Series

# 3.3.1 Pinouts for Universal Robot Tool Connector

![](images/2208767930191ea0d7ebed319794b7fc3f3b303476f5fc74c9fbae58c2e8f3f6.jpg)

<table><tr><td rowspan=1 colspan=1>Pin</td><td rowspan=1 colspan=1>Color</td><td rowspan=1 colspan=1>Signal</td></tr><tr><td rowspan=1 colspan=1>5</td><td rowspan=1 colspan=1>Gray</td><td rowspan=1 colspan=1>24Vdc</td></tr><tr><td rowspan=1 colspan=1>8</td><td rowspan=1 colspan=1>Red</td><td rowspan=1 colspan=1>GND</td></tr><tr><td rowspan=1 colspan=1>1</td><td rowspan=1 colspan=1>White</td><td rowspan=1 colspan=1>RS485+(A)</td></tr><tr><td rowspan=1 colspan=1>2</td><td rowspan=1 colspan=1>Brown</td><td rowspan=1 colspan=1>RS485-(B)</td></tr></table>

Table 3.2: Pinouts for Universal Robot Tool Connector

# 3.3.2 Pinouts of Rainbow Robotics Tool Connector

![](images/e22c8846b58414e3bec782c5c4ce93907110bdbdf2e2495a809c9799a45165ec.jpg)

<table><tr><td rowspan=1 colspan=1>Pin</td><td rowspan=1 colspan=1>Color</td><td rowspan=1 colspan=1> Signal</td></tr><tr><td rowspan=1 colspan=1>3</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>24Vdc</td></tr><tr><td rowspan=1 colspan=1>4</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1> GND</td></tr><tr><td rowspan=1 colspan=1>9</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>RS485+(A)</td></tr><tr><td rowspan=1 colspan=1>10</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>RS485-(B)</td></tr></table>

Table 3.3: Pinouts for Rainbow Robotics Tool Connector

# 3.3.3 Pinouts for Doosan Robotics Tool Connector

<table><tr><td>5 6 4 7 3 8 1 2 A Code male 8 pin</td><td>5 4 6 3 7 8 2 1 A Code female 8 pin</td></tr></table>

<table><tr><td rowspan=1 colspan=1>Pin</td><td rowspan=1 colspan=1>Color</td><td rowspan=1 colspan=1>Signal</td></tr><tr><td rowspan=1 colspan=1>5</td><td rowspan=1 colspan=1>Gray</td><td rowspan=1 colspan=1>24Vdc</td></tr><tr><td rowspan=1 colspan=1>8</td><td rowspan=1 colspan=1>Red</td><td rowspan=1 colspan=1>GND</td></tr><tr><td rowspan=1 colspan=1>1</td><td rowspan=1 colspan=1>White</td><td rowspan=1 colspan=1>RS485+(A)</td></tr><tr><td rowspan=1 colspan=1>2</td><td rowspan=1 colspan=1>Brown</td><td rowspan=1 colspan=1>RS485-(B)</td></tr></table>

Table 3.4: Pinouts for Rainbow Robotics Tool Connector

# 4. LED Indications

그리퍼 LED는 현재 그리퍼 상태를 나타냅니다. 아래 그림의 녹색 원은 LED의 위치를나타냅니다.

![](images/33c1ce4fafaff51d2bfd66a758e1b98b77bcffccfaa64447d80627c15a0d4bd4.jpg)  
Figure 4.1. Position of LED according to Grippers

각 LED 창은 녹색, 붉은색 두가지 색을 표시하며, 동시에 두 색이 같이 표시할수있습니다. 각 색의 상태가 나타내는 정보는 아래와 같습니다. LED 점멸 패턴의 변화주기는 0.5초입니다.

Table 4.1. Gripper Status from Gripper LED   

<table><tr><td rowspan=1 colspan=1>LED</td><td rowspan=1 colspan=1>LED EH</td><td rowspan=1 colspan=1>     ?                 </td></tr><tr><td rowspan=4 colspan=1>片</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>啤</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=2 colspan=1>加</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>H圳品</td></tr><tr><td rowspan=1 colspan=1>叫啤</td><td rowspan=1 colspan=1></td></tr></table>

# 5. Communication

# 5.1 Modbus RTU

씬그립 Optimum 그리퍼는 로봇 또는 자동화 장비와 통신을 하기 위해 RS485 물리신호를 사용하는 모드버스 RTU 표준 프로토콜을 사용합니다. 모드버스 RTU프로토콜은 공개되어 있으며 다음 링크에서 프로토콜 정보를 구할 수 있습니다.

모드버스 RTU 참고 링크:https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf

Optimum 그리퍼와 모드버스 통신을 하기 위한 UART 설정과 사용하는 모드버스프로토콜 설정은 아래와 같습니다.

Table 5.1: Configuration of UART and Modbus RTU   

<table><tr><td rowspan=5 colspan=1>UART  </td><td rowspan=1 colspan=1>Property</td><td rowspan=1 colspan=1>Value</td></tr><tr><td rowspan=1 colspan=1>Baud Rate</td><td rowspan=1 colspan=1>115,200 bps</td></tr><tr><td rowspan=1 colspan=1>Data Bits</td><td rowspan=1 colspan=1>8</td></tr><tr><td rowspan=1 colspan=1> Stop Bits</td><td rowspan=1 colspan=1>1</td></tr><tr><td rowspan=1 colspan=1>Parity</td><td rowspan=1 colspan=1>                            None</td></tr><tr><td rowspan=2 colspan=1>Modbus</td><td rowspan=1 colspan=1> Support Functions</td><td rowspan=1 colspan=1>7       Read Input Registers (FC04)?          Preset Single Register (FC06)Preset Multiple Registers (FC16)</td></tr><tr><td rowspan=1 colspan=1>Modbus ID</td><td rowspan=1 colspan=1>1</td></tr></table>

# Infomation

Modbus의 송수신 기능(Support Functions)에서 그리퍼의 설정 값을 지령 할경우는 Read Single Register를 사용하고, 그리퍼 동작을 위한 지령은 PresetMultiple Registers를 사용할 것을 권고합니다.

Preset Multiple Registers는 동작을 위한 지령 및 그 동작을 위한 매개변수를하나의 CRC로 계산하여 송수신하므로 부분적으로 통신 오류가 발생되었을경우에도 전달되는 정보의 부분만 전달되는 경우를 막을 수 있기 때문입니다

# Infomation

Modbus ID는 디폴트로 1로 설정이 되어 있습니다. 복수의 그리퍼를 이용하기 위한Modbus ID 변경을 위해서는 제조사(www.seengrip.com)으로 연락바랍니다.

![](images/5fd5cb2d212f3b9ef0dfc04d744b0be39b680319df59e1d4779399ba441df661.jpg)

# 5.2 Register Mapping

모드버스 RTU는 클라이언트(로봇 팔)가 서버(그리퍼)에게 데이터를 쓰거나 읽는요청을 통해 송수신 합니다.

# Caution

모드버스 RTU 프로토콜에서는 서버(그리퍼)에서 이벤트 발생시 클라이언트(로봇팔)로 사건을 전달할 수 없습니다. 따라서, 클라이언트(로봇 팔)는 주기적으로서버(그리퍼)의 데이터를 읽어서 상태나 위치 값을 알아내야 합니다.

아래 표의 Holding Register는 그리퍼로 주는 설정 또는 동작 명령이고, InputRegister는 그리퍼로부터 받는 데이터입니다.

<table><tr><td colspan="1" rowspan="1">Register</td><td colspan="1" rowspan="1">Address</td><td colspan="1" rowspan="1">Functionalities</td><td colspan="1" rowspan="1">Value Range</td><td colspan="1" rowspan="1">Units</td></tr><tr><td colspan="1" rowspan="14">Holding</td><td colspan="1" rowspan="1">5</td><td colspan="1" rowspan="1">Limit of Acceleration</td><td colspan="1" rowspan="1">   0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">6</td><td colspan="1" rowspan="1">Grip Torque</td><td colspan="1" rowspan="1">O 0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">7</td><td colspan="1" rowspan="1"> Motion Torque</td><td colspan="1" rowspan="1">1~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">8  C</td><td colspan="1" rowspan="1">1  Limit of Speed</td><td colspan="1" rowspan="1">1~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">19</td><td colspan="1" rowspan="1">     Grip Command</td><td colspan="1" rowspan="1">1</td><td colspan="1" rowspan="1">1</td></tr><tr><td colspan="1" rowspan="1">?16</td><td colspan="1" rowspan="1"> Fault Clear</td><td colspan="1" rowspan="1">1</td><td colspan="1" rowspan="1">1</td></tr><tr><td colspan="1" rowspan="1">17</td><td colspan="1" rowspan="1">Release Retry Time</td><td colspan="1" rowspan="1">0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">18</td><td colspan="1" rowspan="1"> Release Retry Range</td><td colspan="1" rowspan="1">0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">19</td><td colspan="1" rowspan="1">?     GripDriveTime</td><td colspan="1" rowspan="1">0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">20</td><td colspan="1" rowspan="1">GripDrive FRange</td><td colspan="1" rowspan="1">0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">21</td><td colspan="1" rowspan="1">Homing FRetry Count</td><td colspan="1" rowspan="1">1~1000</td><td colspan="1" rowspan="1">[Count]</td></tr><tr><td colspan="1" rowspan="1">22</td><td colspan="1" rowspan="1">Detect Speed</td><td colspan="1" rowspan="1">0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">23</td><td colspan="1" rowspan="1">Flash Write</td><td colspan="1" rowspan="1">1</td><td colspan="1" rowspan="1">-</td></tr><tr><td colspan="1" rowspan="1">62</td><td colspan="1" rowspan="1">Comm Error Check</td><td colspan="1" rowspan="1">0~2</td><td colspan="1" rowspan="1">1</td></tr><tr><td colspan="1" rowspan="1"></td><td colspan="1" rowspan="1">64</td><td colspan="1" rowspan="1"> Time Duration for CurrentMeasurement During SpeedControl</td><td colspan="1" rowspan="1">1~65535</td><td colspan="1" rowspan="1">[ms]</td></tr><tr><td colspan="1" rowspan="10">Input</td><td colspan="1" rowspan="1">2</td><td colspan="1" rowspan="1">Gripper Info</td><td colspan="1" rowspan="1">1</td><td colspan="1" rowspan="1">1</td></tr><tr><td colspan="1" rowspan="1">10</td><td colspan="1" rowspan="1">PresentPosition</td><td colspan="1" rowspan="1">-100~1100</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">11</td><td colspan="1" rowspan="1">Present Speed</td><td colspan="1" rowspan="1">0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">12</td><td colspan="1" rowspan="1">Grip Position [ Deviation</td><td colspan="1" rowspan="1">0~1000</td><td colspan="1" rowspan="1">[0.1%]</td></tr><tr><td colspan="1" rowspan="1">13</td><td colspan="1" rowspan="1"> Present Temperature</td><td colspan="1" rowspan="1">0~2000</td><td colspan="1" rowspan="1">[0.1deg]</td></tr><tr><td colspan="1" rowspan="1">14</td><td colspan="1" rowspan="1">Present Power Voltage</td><td colspan="1" rowspan="1">0~400</td><td colspan="1" rowspan="1">[0.1V]</td></tr><tr><td colspan="1" rowspan="1">15</td><td colspan="1" rowspan="1">Present (t Gripper  Status</td><td colspan="1" rowspan="1">、</td><td colspan="1" rowspan="1">、</td></tr><tr><td colspan="1" rowspan="1">16</td><td colspan="1" rowspan="1">Present Fault</td><td colspan="1" rowspan="1">、</td><td colspan="1" rowspan="1">、</td></tr><tr><td colspan="1" rowspan="1">23</td><td colspan="1" rowspan="1">Flash Write (Count</td><td colspan="1" rowspan="1">0~65535</td><td colspan="1" rowspan="1">[write]</td></tr><tr><td colspan="1" rowspan="1">65</td><td colspan="1" rowspan="1">Average ( Current duringSpeed Control</td><td colspan="1" rowspan="1">0~65535</td><td colspan="1" rowspan="1">[mA]</td></tr></table>

Table 5.2 Register Mapping of Optimum Series

# 5.2.1 Limit of Acceleration (Address 5, Configuration)

그리퍼를 이동할 때 가속도을 제한하지 않으면 설정된 토크 한계 내에서 급격한가속을 하게되어 그리퍼의 진동을 발생시킵니다.

이러한 진동을 감소하기 위해서 Limit of Acceleration을 설정하여 가속도 제한을설정할 수 있습니다.

$0 \%$ 로 설정할 경우에는 가장 느린 가속을 수행하게되고, $100 \%$ 를 설정할 경우에는가장 빠른 가속을 수행하게 됩니다. 초기값에서 특별한 요구가 없을 경우에는변경하지 않는 것을 권고합니다.

Limit of Acceleration 을 수정한 후 Flash에 저장하면 다음 전원 인가 시점에도 설정된값이 유지됩니다.

# Infomation

앞으로 소개될 토크 제한에 의해서도 가속도를 제한 할 수 있습니다. 그러나 토크제한은 모션 도중 발생 가능한 토크 제한 용도로 사용하고, 가속도를 제한 하기위해서는 Limit of Acceleration을 사용하시기를 바랍니다.

# 5.2.2 Grip Torque (Address 6, Command Parameter)

파지 동작을 수행할 경우 속도 이동으로 대상물을 찾아서 정지한 후 주어진 GripTorque에 의해 대상물에 힘을 가하여 파지 동작을 수행합니다.

Grip Command로 ‘Grip’ 또는 ‘Speedy Grip’ 명령에서만 Grip Torque가 유효합니다.

Grip Torque의 범위는 $0 { \sim } 1 0 0 \% 0$ 며 $100 \%$ 는 그리퍼의 발생 가능한 최대 파지력을발생 시킵니다.

Grip Torque 전원이 인가되면 $25 \%$ 로 초기화 됩니다.

# 5.2.3 Motion Torque (Address 7, Command Parameter)

Motion Torque는 위치 이동 또는 속도 이동 시에 발생 할 경우 모터에서 발생할 수있는 최대 Torque의 한계를 설정합니다.

위치 이동 또는 속도 이동이 완료 된 후 파지력 생성을 위한 토크를 발생 시킬 때의토크는 Motion Torque에 의해 제한 되지 않고, Grip Torque에 의해 제한됩니다.

Motion Torque 전원이 인가되면 $50 \%$ 로 초기화 됩니다.

# Information

일반적으로는 최대 가속도를 설정 하더라도 외부의 별도 부하가 없을 경우에는$2 0 \% { \sim } 3 0 \% { \stackrel { \circ } { \to } } 1$ 토크로도 최대 가속을 수행 할 수 있습니다.  
$10 \% 0 |$ 하로 설정하게 되면 모터로 움직이는데 충분한 힘을 발생하지 못해 목표속도 또는 목표 위치에 이르지 못하고 마치 대상물을 감지한 것으로 인식할 수있습니다.  
$50 \% 0 \%$ 상 너무 큰 토크 설정은 핑거와 대상물과의 충돌을 감지하는 순간에 필요이상으로 대상물에 충격을 가할 수 있습니다.  
따라서 특별한 경우를 제외하고는 $20 \sim 5 0 \%$ 범위로 설정하는 것을 권고합니다.

# 5.2.4 Limit of Speed (Address 8, Command Parameter)

Limit of Speed는 위치 이동시 발생 가능한 최대 속도를 설정합니다.

Grip Command로 ‘Move’ 명령과 ‘Speedy Grip’에서 위치 이동 중에서만 Limit ofSpeed가 유효합니다. ‘Grip’ 명령에서는 Grip Command 내의 속도 명령이 발생하는최대 속도가 되며, ‘Speedy Grip’ 명령에서는 목표 위치 도달 후 물체를 탐지하기 위한속도 이동시에는 Detect Speed가 발생가능한 최대 속도가 됩니다.

Limit of Speed의 범위는 $0 { \sim } 1 0 0 \% 0 |$ 며 $100 \%$ 일 경우 그리퍼가 발생 가능한 최대속도가 됩니다. .

Limit of Speed는 전원이 인가되면 $50 \%$ 로 초기화 됩니다.

# Information

그리퍼의 속도는 성능 차트에서 볼수 있듯이 파지력을 결정하는 요인이 됩니다.  
따라서 응용에 따라 필요로 하는 파지력에 맞추어 설정하는 것이 바람직합니다.  
그러나 ‘Move’ 명령으로 높은 속도로 파지를 하게 되면 파지 대상물이 손상되거나그리퍼에서 끼임 현상이 발생할 수 있으며 그리퍼 노화 속도를 가속화 시킵니다.  
따라서 파지시에는 ‘Grip’ 명령 또는 ‘Speedy Grip’ 명령을 사용해야 합니다.

# Information

$10 \% 0 \%$ 하로 설정할 경우 목표 속도 에 이르지 못하고 마치 대상물을 감지한 것으로인식할 수 있습니다. 따라서 특별한 경우를 제외하고는 $10 \% 0 |$ 상으로 설정하는것을 권고합니다.

# 5.2.5 Grip Command (Address 9, Command)

Grip Command를 그리퍼가 받게 되면 아래의 4가지 동작 중 하나를 실행합니다.

Table 5.3 Type of Grip Command   

<table><tr><td rowspan=1 colspan=1>Home</td><td rowspan=1 colspan=1>1</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>Move</td><td rowspan=1 colspan=1>2</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>Grip</td><td rowspan=1 colspan=1>5</td><td rowspan=1 colspan=1>呂</td></tr><tr><td rowspan=1 colspan=1>Speedy Grip</td><td rowspan=1 colspan=1>6</td><td rowspan=1 colspan=1></td></tr></table>

Grip Command는 16bits 정수로 이루어지며 아래의 구성입니다.

Table 5.4 Contents of Grip Command   

<table><tr><td rowspan=1 colspan=1>出里</td><td rowspan=1 colspan=1>15~12 </td><td rowspan=1 colspan=1>      11~0</td></tr><tr><td rowspan=1 colspan=1>咖</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>臣(0.1%</td></tr><tr><td rowspan=1 colspan=1>业</td><td rowspan=1 colspan=1>1,2,5,6</td><td rowspan=1 colspan=1>-1000~1000</td></tr></table>

일반적으로 코딩으로 두 정보를 하나의 정보로 모을수 있습니다. 여기서 매개변수는Signed 12bits 정수입니다.

Grip Command $=$ ((명령번호 <<12)&0xF000) | (매개변수 & 0x0FFF)

A. Home (명령 번호: 1, 매개변수: 원점복귀 속도)

Optimum 시리즈는 핑거에 엔코더가 장착되어 전원을 인가하면 초기 위치를 알수있어 자동 원점 복귀를 수행합니다.

그러나 핑거 엔코더는 정밀도가 $2 \%$ 수준이므로 응용에 따라 높은 정밀도를 요구하는경우 전원이 다시 인가할 때마다 원점 복귀를 수행할 필요가 있습니다.

원점 복귀 동작 지령의 매개변수는 원점 복귀 속도입니다. 원점 복귀 속도의 범위는$0 { \sim } 1 0 0 \%$ 로, $0 \%$ 일 경우는 원점 복귀 최저 속도(그리퍼 최대 속도의 $10 \%$ ) , $100 \%$ 일경우에는 원점 복귀 최대 속도(그리퍼 최대 속도의 $50 \%$ )를 지정할 수 있습니다.

# Information

사용설정 및 사용환경에 따라 원점 복귀를 한번에 수행하지 못하는 경우가있습니다. 이 경우를 위해 Homing Retry Count 만큼 원점 복귀를 수행하게 됩니다.

# Caution

그리퍼는 종류 마다 하드웨어적으로 동작 범위가 정해져 있습니다. 따라서 원점복귀 수행 후 하드웨어 동작 범위만큼 이동하지 못했을 경우에는 원점 복귀가실패한 것으로 판단하고 오류 메시지를 발생합니다.  
원점 복귀가 실패한 이후에는 현재 위치가 불확실 하므로 위치 이동 지령, 위치이동 후 토크 발생 지령을 수행 할 수 없으며, 단지 속도 이동 지령, 속도 이동 후토크 발생 지령만 가능합니다.

# B. Move (명령 번호: 2, 매개변수: 목표 위치)

그리퍼의 위치 이동을 위해 사용하는 명령입니다. 목표 위치 범위는 $0 \{ 1 0 0 \%$ 범위이며, $0 \%$ 일 경우 완전히 핑거가 닫히게 되고, $100 \%$ 일 경우에는 핑거가 완전히열리게 됩니다.

위치 이동에는 목표 위치 이외에 Motion Torque, Limit of Speed를 매개변수로사용합니다. Grip Command와 함께 전달할 수도 있고, 이미 설정된 Motion Torque,Limit of Speed을 사용하는 경우는 Motion Torque, Limit of Speed를 전달하지 않아도됩니다.

위치 이동은 그리퍼의 이동 전 상태를 알 수가 없습니다. 이전 상태가 강한파지력으로 대상물을 파지한 상태일 수 있습니다. 강하게 잠겨진 핑거를 풀기 위해위치 이동 이전에 Release 동작을 수행합니다. Release 동작은 위치 목표를 도달하기전에 핑거가 멈춘다면 그리퍼의 모터가 발생 가능한 최대 토크를 이용하여 파지한 힘이상을 반복적으로 발행하여 모터를 이동시킵니다.

# Information

Release 동작에서 최대 토크를 활용하는 조건은 다음과 같습니다.

1. 모터의 온도가 100도 이하일 경우  
2. 정격 전압 입력과 현재 입력 전압의 오차가 $16 \%$ 를 이하인 경우  
3. 설정된 Release Retry Time을 이내인 경우  
4. 설정된 Release Retry Range를 이내인 경우

모터를 움직이려 했을때 모터가 이동하지 않으면 위의 조건이 만족되는 한 모터의최대 토크를 활용하여 이동하기 위한 시도를 반복하게 됩니다.  
위의 1 또는 2가 아닌 경우 모터가 이동하지 않으면 오류를 발생하고 더 이상모터를 동작하기 위한 시도를 하지 않습니다.  
만약 3 또는 4가 아닌 경우 모터가 이동하지 않으면 위치 이동을 하다 예상하지않은 장애물을 만난 것으로 간주하여 위치에 도달하지 않더라도 위치 이동을 위한시도를 더 이상 하지 않습니다.

# Caution

그리퍼 동작에서 ‘Move’ 명령으로 목표 위치이전 위치에 대상물과 충돌이 일어나는파지를 시도하는 것은 바람직하지 않습니다.  
파지를 위해서는 ‘Grip’ 또는 ‘Speedy Grip’ 명령을 사용해야 안정적인 파지력을확보할 수 있습니다.  
‘Move’ 명령은 목표 위치까지 장애물이 없다는 전제로 사용해야 합니다. 그럼에도불구하고 예외 경우는 있을 수 있으므로 목표 위치 이동 지령 후 동작이 완료된 후현재 위치나 상태를 읽어서 동작 완료 여부를 확인하는 것이 안정적인 시스템을구축하는 방법입니다.

![](images/da1960e2de37633bee47a8fe2c7c78bca0fb81a8f28cae7c60cef3602375fe8e.jpg)

# C. Grip (명령 번호: 5, 매개변수: 목표 속도)

Grip 명령은 일반적으로 그리퍼로 물체를 파지할 때 사용합니다. 매개변수로 입력받은 목표 속도로 이동하며 파지 대상물을 찾습니다. 대상물과 충돌이 발생되면대상물을 감지한 것으로 판단하여 정지합니다. 정지 이후 Grip Torque에 해당하는파지력을 발생합니다.

대상물이 없는 경우 하드웨어 한계까지 주어진 속도를 발생시킵니다.

매개변수는 목표 속도를 나타냅니다. $- 1 0 0 { \sim } 1 0 0 \%$ 범위를 가집니다. 매개변수가 음일경우는 핑거가 닫히는 방향으로, 양일 경우는 핑거가 열리는 방향으로 최대 속도로이동합니다.

# Information

$20 \% 0 \%$ 하로 설정할 경우 목표 속도 에 이르지 못하고 마치 대상물을 감지한 것으로인식할 수 있습니다. 따라서 특별한 경우를 제외하고는 $20 \%$ 이상으로 설정하는것을 권고합니다.

물체를 탐지하기 위해 속도 이동에는 Motion Torque에 의해 최대 토크가 제한되며,파지력은 Grip Torque에 의해 결정됩니다.

파지력 발생은 Grip Drive Time과 Grip Drive Range에 의해 제한됩니다. Grip DriveTime은 파지력을 발생하는 최대 시간으로 만약 파지력을 발생시키는 동안 핑거의움직임이 멈추지 않는다면 Grip Drive Time 시간이 지나면 파지력 발생은 멈춥니다.Grip Drive Range는 파지력 발생 후 Grip Drive Range 이상으로 이동이 된다면파지력 발생을 멈춥니다.

대상물을 감지하여 정지 한 후 주어진 Grip Torque를 발생시켜 움직이는 거리는 GripPosition Deviation에 저장됩니다. 이 Grip Position Deviation을 통해서 파지한대상물이 얼마나 단단한지를 확인할 수 있습니다.

# D. Speedy Grip (명령 번호: 6, 매개변수: 목표 위치)

Grip 명령로 물체를 탐지하는 속도는 그리퍼가 발생가능한 최대 속도 이하이기때문에 먼 거리를 이동할 때에는 시간 소모가 클 수 있습니다. 이러한 단점을보완하기 위한 명령이 ‘Speedy Grip’입니다.

‘Speedy Grip’은 ‘Move’와 ‘Grip’ 명령을 순차적으로 수행합니다.대상물이 없다고 판단된는 안전 위치까지는 ‘Move’ 명령의 위치이동을 수행하고,대상물이 있을 수 있는 위치부터는 ‘Grip’ 명령으로 안정적인 속도로 대상물을 탐지한후 파지력을 발생합니다.

위치 이동 방식은 ‘Move’ 명령과 동일하므로 Release Retry Time과 Release RetryRange에 의해 목표 위치까지의 이동을 보장받을 수 있습니다.

파지 방식은 ‘Grip’ 명령과 동일하므로 Grip Drive Time과 Grip Drive Range에 의해파지력 발생이 제한되며, 파지력 발생으로 움직인 거리는 Grip Position Deviation에저장됩니다.

# 5.2.6 Fault Clear (Address 16, Configuration)

그리퍼에서 오류가 발생되면 LED로 오류 상황을 알리고, Fault Status에 최근 오류상황을 쓰게 됩니다. 해당 Fault Status에 따라 오류를 해결한 후 Fault Clear레지스터에 어떤 값이든 쓰기 동작을 수행하면 LED는 정상으로 돌아오고, FaultStatus 값은 지워집니다.

# 5.2.7 Release Retry Time (Address 17, Configuration)

‘Move’ 명령을 수행할 때 목표 위치까지 이동을 수행을 시작할 때, 강하게 잠겨 있는파지 상태를 Release하기 위해 그리퍼 모터의 최대 토크를 활용하는 시간을설정합니다.

위치 재시도 시간의 범위는 $0 \{ 1 0 0 \%$ 이며, $100 \% \equiv$ 최대 시간인 10초를 나타냅니다.

‘Move’ 명령으로 목표 위치까지 이동을 보장하고 싶을 때에는 $100 \%$ 를 설정하는 것을권고합니다.

Release 운영을 하고 있는지 유무는 Grip Status의 동작 수행 상태를 통해 확인가능합니다.

Release Retry Time을 수정한 후 Flash에 저장하면 다음 전원 인가 시점에도 설정된값이 유지됩니다.

# 5.2.8 Release Retry Range (Address 18, Configuration)

‘Move’ 명령을 수행할 때 표 위치까지 이동을 수행을 시작할 때, 강하게 잠겨 있는파지 상태를 Release하기 위해 그리퍼 모터의 최대 토크를 활용하는 위치 범위를설정합니다.

위치 재시도 거리 범위는 $0 { \sim } 1 0 0 \%$ 이며, $100 \%$ 는 해당 그리퍼의 최대 이동 거리를나타내며 그리퍼 마다 다른 값을 가집니다. 초기값은 약 $10 \%$ 수준으로 설정되어

있으며, 목표 위치까지 도달을 보장 받기 위해서는 $100 \%$ 로 설정하면 되며, 일정 거리만큼만 위치 이동을 보장 받기 위해서는 해당 위치 범위를 설정하면 됩니다.

현재 위치에서 목표 위치까지의 거리가 위치 재시도 거리 범위를 미만일 경우는 현재위치에서 목표 위치까지 거리가 우선하게 됩니다.

Release Retry Range 을 수정한 후 Flash에 저장하면 다음 전원 인가 시점에도설정된 값이 유지됩니다.

# 5.2.9 Grip Drive Time (Address 19, Configuration)

‘Grip’ 또는 ‘Speedy Grip’ 명령에서 파지 대상물을 찾은 후 파지력을 발생시키는시간을 설정합니다.

토크 발생 시간의 설정 범위는 $0 { \sim } 1 0 0 \% 0 |$ 며, $100 \%$ 는 50[ms]를 나타냅니다. 초기값은$100 \%$ 로 설정되어 있습니다.

# Information

토크 발생 최대 시간 50[ms]는 그리퍼 모터 토크 시상수에 비해 상대적으로 아주 큰값입니다.

토크 발생 최대 시간 설정은 응용에 따라, 일반적으로 부드러운 대상물을 파지하는경우 높은 값을 딱딱한 대상물을 파지하는 경우는 낮은 값을 사용합니다.

Grip Drive Time을 수정한 후 Flash에 저장하면 다음 전원 인가 시점에도 설정된 값이유지됩니다.

# 5.2.10 Grip Drive Range (Address 20, Configuration)

‘Grip’ 또는 ‘Speedy Grip’ 명령에서 파지 대상물을 찾은 후 파지력을 발생하는 거리한계를 설정합니다.

토크 발생 거리 한계 설정 범위는 $0 { \sim } 1 0 0 \% 0 |$ 며, $100 \%$ 는 해당 그리퍼의 최대 이동거리를 나타내며 그리퍼 마다 다른 값을 가집니다. 초기값은 $10 \%$ 로 설정되어있습니다.

Grip Drive Range을 수정한 후 Flash에 저장하면 다음 전원 인가 시점에도 설정된값이 유지됩니다.

# Information

토크 발생 거리 한계를 설정 했다고 해서, 한계 거리에 모터가 정확하게 멈추는것은 아닙니다. 최종 이동 거리는 그리퍼 또는 핑거의 관성에 의해 결정됩니다.

# 5.2.11 Home Retry Count (Address 21, Configuration)

Home Retry Count는 원점 복귀 실패할 경우 반복하는 횟수를 설정합니다.

최대 원점 복귀 시도 회수 의 단위는 반복 회수이며, 초기 값은 3으로 설정되어있습니다.

Home Retry Count을 수정한 후 Flash에 저장하면 다음 전원 인가 시점에도 설정된값이 유지됩니다.

# 5.2.12 Detect Speed (Address 22, Configuration)

‘Speedy Grip’ 명령에서 주어진 위치에 도달한 후 대상물을 탐지할 때 사용하는 속도한계를 나타냅니다.

‘Grip’ 명령에서는 Grip Command의 매개변수로 대상물을 탐지하는 속도를지정하지만, ‘Speedy Grip’ 명령에서는 Grip command의 매개변수는 목표위치이기때문에 본 주소로 대상물을 찾는 속도를 지정합니다.

단위는 $\%$ 를 사용하고, $0 { \sim } 1 0 0 \%$ 범위를 가집니다. 움직이는 방향은 목표 위치를 위해움직이는 방향을 가집니다.

# 5.3.13 Flash Write (Address 22, Configuration)

원점 복귀를 시행 결과 또는 각종 설정을 Flash에 저장할 때 사용하는 명령입니다.

해당 주소에 어떤 값이든 쓰게되면 Flash에 해당 정보를 저장하고 그리퍼를재부팅합니다.

# Caution

Flash 메모리는 쓰기 수명이 한정적이기 때문에 매번 로봇이나 자동화 장비를 켤때마다 Flash에 쓰는 동작을 할 경우 그리퍼의 수명을 저하 할 수 있으므로 필요한경우에만 사용하여야 합니다.

# 5.3.14 Comm Error Check (Address 62, Configuration)

로봇에 따라 CRC 루틴을 로봇 언어로 구현하기 힘든 경우가 있습니다. 이 경우를위하여 그리퍼의 통신 오류 검출 방법 변경이 가능합니다.

<table><tr><td rowspan=1 colspan=1>Comm Error Check {</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>0</td><td rowspan=1 colspan=1>Modbus CRC</td></tr><tr><td rowspan=1 colspan=1>1</td><td rowspan=1 colspan=1>CheckSum (Modbus CRC 2 Il CheckSum ≌)</td></tr><tr><td rowspan=1 colspan=1>2</td><td rowspan=1 colspan=1>  (Mdbus CRC  )</td></tr></table>

통신 오류 검출 방법 변경은 Comm Error Check 주소에 해당 방식을 저장하고,Flash에 저장한 후, 다음 로봇 부팅부터 유효합니다.

CheckSum의 경우 Modbus 프로토콜에서 0 Byte부터 CRC 이전 Byte까지 합을CheckSum(16Bits 저장소)에 저장한 후 CRC High Byte 공간에 CheckSum High Byte값을 저장하고, CRC Low Byte 공간에 CheckSum Low Byte 값을 저장합니다.

# Caution

통신 오류 검출 방식중 CRC 방식이 가장 우수한 오류 검출 능력을 가지고있습니다. 따라서 특별한 경우가 아니면 표준 Modbus 방식인 CRC 방식을사용하셔야 합니다. CheckSum 이나 ‘검출 안함’ 방식을 사용하면 통신 오류를검출하지 못할수 있다는 것을 인지한 경우만 사용하셔야 합니다.

# Caution

통신 오류 검출 방식이 바꾸고 그리퍼 재부팅이 된 이후부터는 이전에 사용하셨던통신 오류 검출 방식으로는 통신이 불가합니다. 따라서 통신 오류 검출 방식변경에는 상당한 주의가 필요합니다. 특정 로봇의 경우 통신 방법 변경이 어려울 수있으므로, 컴퓨터를 이용하여 통신 변경 및 통신 알고리즘을 충분히 검증 하신 후사용하셔야 합니다.

# 5.3.15 Gripper Information (Address 2, Status)

Gripper Information은 그리퍼의 종류, 하드웨어 버전, 펌웨어 버전을 나타냅니다.

16bits 공간을 아래와 같이 나누어서 내용을 표현합니다.

<table><tr><td rowspan=1 colspan=1>bits</td><td rowspan=1 colspan=1>15~8</td><td rowspan=1 colspan=1>7~4</td><td rowspan=1 colspan=1>3~0</td></tr><tr><td rowspan=1 colspan=1>咖</td><td rowspan=1 colspan=1>臣</td><td rowspan=1 colspan=1>臣</td><td rowspan=1 colspan=1>呂否异1: Type R     2:Type A3: Type P</td></tr></table>

# 5.3.16 Present Position (Address 10, Status)

Present Position은 그리퍼의 현재 위치를 읽을때 사용합니다.

단위는 위치 설정과 동일합니다. 범위는 $0 { \sim } 1 0 0 \% \ 0 |$ 며 그리퍼가 최대로 열렸을 때$100 \% 0 \%$ 며, 그리퍼가 완전히 닫혔을 때 $0 \%$ 를 가집니다.

원점 복귀 이후에 유효한 값이며, 원점 복귀가 되지 않은 상태에서는 올바르지 않은값이 읽힐 수 있습니다.

# 5.3.17 Present Speed (Address 11, Status)

Present Speed는 그리퍼의 현재 속도를 읽을때 사용합니다.

단위는 속도 설정과 동일합니다. 범위는 $0 { \sim } 1 0 0 \% \mathrm { ~ 0 ~ }$ 며 그리퍼의 최대 속도 일때$100 \% 0 \%$ 며, 그리퍼가 정지되었을 때 $0 \%$ 를 가집니다.

# 5.3.18 Grip Position Deviation (Address 12, Status)

‘Grip’ 또는 ‘Speedy Grip’의 경우 대상물 탐지 후 파지력을 발생할때 이동한 거리를나타냅니다. 대상물을 찾고 완전히 멈춘 상태에서 토크를 발생 시켜 움직인거리입니다.

그리퍼의 최대 스트로크가 $100 \% 0 \%$ 며, 움직이지 않는 것이 0%입니다.

Grip Position Deviation은 대상물이 얼마나 유연한지를 확인 할 수 있습니다. 유연한대상물 일 경우 큰 값이 읽히고, 단단한 대상물일 경우 작은 값이 읽힙니다.

# Information

높은 Motion Torque 값에서는 Grip Torque를 발생시켰을때 움직일 거리가 작을 수있습니다. Grip Position Deviation을 상세히 측정하고 싶을 때는 Limit of Speed와Motion Torque에 작은 값을 설정하는 것이 유리합니다. $2 0 \%$ 수준)

# Information

Grip Position Deviation은 핑거 엔코더가 없는 경우 이전 전원 인가시 수행한 최종상태를 $0 \mathsf { f } \equiv$ 수단을 제공하는 유용한 값입니다.

그리퍼의 전원이 인가된 후 속도 이동후 토크 발생 명령에서 속도를 $0 . 1 \%$ 또는$- 0 . 1 \%$ 로 방향만 설정한 경우 토크 발생만 수행하게 되는데, 이때 대상물이 이미파지 된 상태라면 Grip Position Deviation이 0에 가까운 값을 가질 것이며, 파지된대상물이 없을 경우에는 Grip Position Deviation이 큰 값을 가지게 됩니다.전원 인가 후 대상물 파지 유무를 확인하기 위해서는 최소한 파지하는 방향이핑거가 열리는 방향으로 대상물을 파지하는지 핑거가 닫히는 방향으로 대상물을파지하는지에 대한 정보는 필요합니다.

# 5.3.19 Present Temperature (Address 13, Status)

그리퍼 모터의 현재 온도를 나타냅니다. 단위는 0.1deg 단위로 표현됩니다.

그리퍼의 모터를 보호하기 위해 Optimum 시리즈 그리퍼는 온도 센서를 내장하고있으며, 모터 온도가 100deg 이상인 경우 이동 명령을 받으면 그 명령을 수행하지않고 오류를 발생 시킵니다.

Optimum 시리즈 그리퍼는 연속적으로 토크를 발생하지 않기 때문에 에너지 소모가작고, 열발생이 작습니다. 따라서 그리퍼가 정상적인 상태와 정상적으로 운용할경우에는 열발생 초과 오류는 발생되지 않습니다.

# Information

열이 발생 되는 부분은 그리퍼의 모터 가속 영역과 위치 이동 명령 및 위치 이동 후토크 발생 명령에서 최대 토크 활용 영역입니다.  
가속 영역에서는 최대 가속도일 경우에도 소모되는 전류가 작고, 가속 중 장애물을만나면 멈추기 때문에 모터의 온도를 크게 높일 만큼의 열을 발생하지는 못합니다.최대 토크 활용 영역은 모터에서 최대 전류를 사용하기 때문에 열발생 가능합니다.그러나 시간과 거리가 제한되어 있으므로, 정상적인 경우에는 오류를 발생할만큼의 열 발생은 없습니다.

# 5.3.20 Present Power Voltage (Address 14, Status)

그리퍼에 공급되는 전압을 나타냅니다. 단위는 0.1V 단위로 표현됩니다.

로봇이나 자동화 장비에서 올바르게 전압을 공급하고 있는지 확인합니다. 만약 정격전압에서 $13 \% 0 |$ 상의 오차가 발생되면, 이동 명령은 수행하지 않고 오류를 발생시킵니다.

# 5.3.21 Present Gripper Status (Address 15, Status)

Present Gripper Status는 그리퍼의 동작 상태를 현재 수행 명령, 그리퍼 상태, 동작수행 상태로 $L t = O H$ 표현합니다.

16bits 내부를 아래와 같이 나누어 표현합니다.

<table><tr><td>bits</td><td>15~12</td><td>11~8</td><td>7~4</td><td>3~0 ?</td></tr><tr><td>Content</td><td>Command 0: Ready 1: Home 2: Move 5: Grip 6: Speedy Grip</td><td>Home State 0: Not Homed 1: Homed</td><td>Run State 0: Ready 1: Positioning 2: Speeding 3. Torquing 4. Homing 5: Fail</td><td>Motion State 0: Ready 1: Before Release 2 After Release 3: Before Detect 4: After Detect </td></tr></table>

Table 5.5 Contents of Present Gripper Status

# Information

동작 수행 상태는 위치 이동, 속도 이동 , 토크 발생 수행 단계를 보다 세부적으로표현합니다.

명령을 받게 되면 먼저 그리퍼가 움직이게 만드는 단계인 Before Release 상태에진입합니다. 이때는 모터에서 발생 가능한 최대 토크를 활용합니다.

Before Release 상태에서 모터의 움직임을 감지하면 After Release 상태로변화하여 주어진 위치 이동, 속도 이동, 토크 발생을 시작하고 Before Detect 상태로진입합니다.

Befor Detect 상태에서 급격한 감속이 발생되거나 모터의 이동이 정지한 상태가유지되면 After Detect 상태에 진입하게 되고,

After Detect 상태에서 모터가 완전히 정지 상태가 확인 되면 그리퍼 상태와 동작수행 상태를 Ready 상태로 진입하고 명령을 종료합니다.

# 5.3.22 Present Fault (Address 16, Status)

그리퍼에서 오류를 감지하면, LED오류 상태를 알리고, Present Fault에 오류 정보를저장합니다. 감지 가능한 오류와 대처 방안은 아래와 같습니다.

Table 5.6 Faults of Gripper   

<table><tr><td rowspan=1 colspan=1>异座</td><td rowspan=1 colspan=1>北</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>1</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>                 ，</td></tr><tr><td rowspan=1 colspan=1>2</td><td rowspan=1 colspan=1>  </td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>3</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>4</td><td rowspan=1 colspan=1> 异</td><td rowspan=1 colspan=1>1</td></tr><tr><td rowspan=1 colspan=1>5</td><td rowspan=1 colspan=1>FLASH </td><td rowspan=1 colspan=1>Flash  </td></tr><tr><td rowspan=1 colspan=1>6</td><td rowspan=1 colspan=1>Modbus Time out</td><td rowspan=1 colspan=1>昙 odbus </td></tr><tr><td rowspan=1 colspan=1>7</td><td rowspan=1 colspan=1>Modbus Invalid Register</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>8</td><td rowspan=1 colspan=1>Modbus Invalid Use</td><td rowspan=1 colspan=1>Modbus </td></tr><tr><td rowspan=1 colspan=1>9</td><td rowspan=1 colspan=1>Flash  </td><td rowspan=1 colspan=1>Flash  </td></tr><tr><td rowspan=1 colspan=1>10</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>11</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>12</td><td rowspan=1 colspan=1>兆</td><td rowspan=1 colspan=1>→</td></tr><tr><td rowspan=1 colspan=1>13</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>→个</td></tr><tr><td rowspan=1 colspan=1>14</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>15</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>16</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr></table>
오류 번호     오류 내용                         대처 방안
1           핑거 엔코더 오류                    부트업 원점 복귀 불가 -> 원점 복귀 후 위치 이동 또는 위치 이동 후 토크 발생 명령 사용 
2           모터 엔코더 오류                    정상적인 그리퍼에서 미발생
3           전류 입력 오류                      정상적인 그리퍼에서 미발생
4           구동 회로 오류                      정상적인 그리퍼에서 미발생
5           FLASH 메모리 오류                   Flash 저장 재시도
6           Modbus Time out                     올바른 modbus 프로토콜 사용
7           Modbus Invalid Register             레지스터 번호 재확인
8           Modbus Invalid Use                  올바른 modbus 프로토콜 사용 
9           Flash 메모리 저장 오류                  Flash 저장 재시도
10          입력 전압 상한 오류                     입력 전압 확인 또는 결선 확인
11          입력 전압 하한 오류                     입력 전압 확인 또는 결선 확인
12          고 온도 오류                        정상적인 그리퍼에서 미발 생, 최대 토크 사용시간이 길 경우 발생 가능-> 이동 가능한 영역으로 위치 이동 명령 수행
13          최대 토크 사용 시간 초과                위치 이동 명령에서 목적 위치에 도달하지 못한 상태에서 제한된 시간이 지난 경우 -> 이동 가능한 영역으로 위치 이동 명령 수행
14          원점 복귀 중 명령 입력                  원점 복귀 도중 타 명령 입력 -> 원점 복귀 후 이동 명령 수행
15          원점 복귀 전 위치 이동 명령             원점 복귀 되지 않은 상태에서 위치 이동 명령 수행한 경우 -> 원점 복귀 수행
16          원점 복귀 최대 횟수 초과                핑거가 완전 닫기 또는 완전 열기 가능하도록 조치한 후 원점 복귀 수행

# 5.3.23 Flash Write Count (Address 23, Status)

Flash Write Count 레지스터를 읽으면 Flash 메모리에 쓰기 횟수를 확인할 수있습니다. Flash 쓰기 횟수를 알려주는 이유는 반복적으로 회수가 증가된다면 조치를취해야 하기 때문입니다.

Flash 메모리는 쓰기 횟수가 제한되어 있으므로 사용시 마다 전원이 들어올때 마다Flash 메모리에 쓰는 명령을 수행하는 것은 바람직하지 않습니다. Flash WriteCount가 계속 증가되는 상황이라면 사용자 프로그램을 확인하시기 바랍니다.

# 5.3.24 Time Duration for Current Measurement during Speed Control (Address 64, Configuration)

Grip 명령 수행시 속도 제어를 하는 동안 소모된 전류의 평균값을 측정할 수있습니다. 속도 제어 시작 후 전류를 측정하는 시간을 본 변수로 제한합니다. 측정시간을 동일한 조건으로 맞추면 외란에 의한 토크 소모량을 보다 정확히 측정가능합니다.

# 5.3.25 Average Current during Speed Control (Address 65, Configuration)

Grip 명령 수행시 속도 제어 시작 후 제한 된 시간 (Time Duration for CurrentMeasurement during Speed Control) 동안 소모된 평균 전류값입니다.

본 값은 핑거 이동시 소모한 전류를 나타내며, 이 값이 클 경우는 파지 대상물이 정중앙에 있지 않은 경우이며, 파지 대상물이 정 중앙에 있을 경우 이 값은 작습니다.

의도적으로 대상물을 한쪽으로 치우쳐 놓은 후, 소모된 평균 전류를 측정하여 파지대상물의 무게를 측정하는데 사용될 수 있습니다.

# 6. How to Use

# 6.1 Grasp Parameters and Procedures

Optimum 그리퍼는 목표 위치로 움직이는 Move, 목표 힘으로 파지 동작을 위한 Grip,빠른 시간 내에 파지까지 수행하기위해 Move와 Grip를 순차적으로 수행하는 SpeedyGrip을 사용할 수 있습니다.

여기서는 각 명령 별로 사용되는 매개변수에 대한 정리와 도식화된 설명을 보여줍니다. ()안의 숫자는 Modbus Address를 나타냅니다.

# 6.1.1 Move

Table 6.1: Parameters of Move Command   

<table><tr><td rowspan=1 colspan=1>Limitation</td><td rowspan=1 colspan=1>Releasing Phase</td><td rowspan=1 colspan=1>Positioning Phase</td></tr><tr><td rowspan=1 colspan=1>Position</td><td rowspan=1 colspan=1>■</td><td rowspan=1 colspan=1>Grip Argument (9)</td></tr><tr><td rowspan=1 colspan=1>Speed</td><td rowspan=1 colspan=1>=</td><td rowspan=1 colspan=1>Limit of Speed (8)</td></tr><tr><td rowspan=1 colspan=1>Torque</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>Motion Torque (7)</td></tr><tr><td rowspan=1 colspan=1>Time</td><td rowspan=1 colspan=1>Release Retry Time (17)</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>Range</td><td rowspan=1 colspan=1>Release Retry Range(18)</td><td rowspan=1 colspan=1>！</td></tr></table>

Move 명령은 핑거에서 움직임이 발생되기까지 Releasing Phase에 있으며 이때그리퍼 모터의 최대 토크를 반복적으로 발생합니다. 핑거에서 움직임이 발생되면매개변수에 맞는 가속, 등속, 감속 과정을 거쳐서 목표 위치에 다 다릅니다. 가속은Motion Torque와 Limit of Acceleration에 의해 제한되고 Limit of Acceleration이우선합니다. 감속은 오버슈팅이 제한된 위치 제어기에 의해 감속을 수행됩니다.

![](images/eaa7a6bfec95a3b86a360a50a0614859916ba9c591c379b74b8879b357a7bba1.jpg)  
Fig. 6.1: Move Command Mechanism

# 6.1.2 Grip

Table 6.2: Parameters of Grip Command   

<table><tr><td>Limitation</td><td>Detecting Phase</td><td>Driving Phase</td></tr><tr><td>Position</td><td>1</td><td>1</td></tr><tr><td>Speed</td><td>Grip Argument (9)</td><td></td></tr><tr><td>Torque</td><td>Motion Torque (7)</td><td>Grip Torque (6)</td></tr><tr><td>Time</td><td></td><td>Grip Drive Time (19)</td></tr><tr><td>Range</td><td>1</td><td>Grip Drive Range (20)</td></tr></table>

‘Grip’ 명령은 주어진 속도로 움직이며 파지 대상물을 찾습니다. 핑거와파지대상물이 충돌하면 핑거를 멈춥니다. 정지 후 Grip Torque를 발생합니다. GripTorque가 발생된 후 움직인 거리는 Grip Position Deviation (12)에 저장되어 대상물의유연도를 계측하여 사용자에게 보여줍니다. Driving Phase에서 계속 움직이지 않도록Grip Drive Time과 Grip Drive Range에 의해 움직임이 제한됩니다.

![](images/f9f4d8048b2de29b4bc340d7d33588feaa33f9a81174480b34d499689de63dc1.jpg)  
Fig. 6.2: Grip Command Mechanism

# 6.1.3 Speedy Grip

<table><tr><td rowspan=1 colspan=1>Limitation</td><td rowspan=1 colspan=1>ReleasingPhase</td><td rowspan=1 colspan=1>PositioningPhase</td><td rowspan=1 colspan=1>DetectingPhase</td><td rowspan=1 colspan=1>Driving Phase</td></tr><tr><td rowspan=1 colspan=1>Position</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>Grip Argument（9)</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>Speed</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>Limit of Speed(8)</td><td rowspan=1 colspan=1>Grip Argument（9)</td><td rowspan=1 colspan=1></td></tr><tr><td rowspan=1 colspan=1>Torque</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>Motion Torque（7)</td><td rowspan=1 colspan=1>Motion Torque（7)       C</td><td rowspan=1 colspan=1>Grip Torque (6)</td></tr><tr><td rowspan=1 colspan=1>Time</td><td rowspan=1 colspan=1>Release RetryTime (17)</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>Grip DriveTime(19)</td></tr><tr><td rowspan=1 colspan=1>Range</td><td rowspan=1 colspan=1>Release RetryRange(18)</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>，</td><td rowspan=1 colspan=1>Grip Drive Range (20)</td></tr></table>

Table 6.1: Parameters of Speedy Grip Command

'Speedy Grip'은 'Grip' 명령의 최대 속도가 그리퍼 발생 속도의 $40 \%$ 로 제한되어 있어파지 시간이 많이 소요된다는 단점을 보완하기 위해서 Move 명령과 Grip 명령을순차적으로 수행합니다.

![](images/76432ed37d1bb5765a51b128ce8ac14c86f1b12cedce031a0b61b9cfeaf36abb.jpg)  
Fig. 6.3: Speedy Grip Command Mechanism

# 6.2 Grasp Applications (Optimum R, A)

Optimum R 또는 Optimum A는 Finger 사이에 공간이 있기 때문에 3 점 지지형태의파지가 가능합니다. 이를 위해 Palm Tool Attach Thread를 이용하여 Palm에 3점지지에 유리한 Tool을 추가 할 수 있습니다.

상세한 3점 지지 방법은 대상물에 따라 여러가지 방법이 있습니다. 여기서는활용하는 예시를 보여줍니다.

Optimum R은 하나의 그리퍼로 다양한 원형 대상물을 3점 지지할 수 있습니다.아래의 예시는 Optimum R로 대상물이 큰 경우와 작은 경우에 대한 파지 예시를보여줍니다.

대상물의 직경이 큰 경우에는 아래의 붉은색 점에서 3 위치에서 접촉이이루어지므로 2위치 접촉보다는 안정적인 파지가 가능합니다.

![](images/db0e5185cc66be7b2c229c89a66d4e3b2d360315690282098d0d68d4606b8c0b.jpg)  
Fig. 6.4: Grasping a Big Cylinder Object with Optimum R

아래의 예시는 작은 실린더형 대상물을 파지하는 예시를 보여주며 접촉 위치만달라질 뿐 3위치 접촉에 의한 파지가 가능합니다.

![](images/7510f5fa0df842db3ecdb897be1f6003d57851a914fde224ba65802cf3a96d68.jpg)  
Fig. 6.5: Grasping a Small Cylinder Object with Optimum R

Optimum R로서 대상물의 중심점 위치를 선정하는 방법은 두 경우 모두 아래의 검은색 테두리가 있는 붉은 점을 기준으로 Z축 방향으로 실린더 원의 반지름 만큼 먼거리가 대상물의 중심 위치가 됩니다.

Optimum A는 항상 평행을 이루는 Finger End만 이용하여 박스형 대상물을 넓은범위에 대해 파지할수 있지만, Finger 사이공간을 이용하면 3점 지지로 사용할 수있도록 설계되어 있습니다.

아래 예시는 Optimum A를 사용하여 박스형 대상물을 파지하는 형태를 보여줍니다.  
예시 그림에서와 같이 다양한 대상물의 크기에 대응 가능합니다.

![](images/f9dd5d842984fdddf1e9d56f2049a7dee74464e3a843105af6eb46a13b680c37.jpg)  
Fig. 6.6: Grasping a Box Object with Optimum A

아래 예시는 Optimum A를 사용하여 원형 대상물을 3점 파지하는 형태를보여줍니다.

![](images/89973a0357be4ac71ee4abdd48357dc0bd344c1d432bc2b885c174e316596ccf.jpg)  
Fig. 6.7: Grasping a Cylinder Object with Optimum A

Optimum A 예시들은 별도의 Finger Tip이 없는 경우의 예시이며, Finger Tip디자인에 따라 보다 많은 응용이 가능합니다.

# 7. Trouble Shooting

정상적인 경우는 트러블 슈팅이 필요 없지만, 극단적인 그리퍼 사용 또는 장시간그리퍼 사용을 하게 되면 그리퍼에 문제가 발생될 수 있습니다. 본 장에서는 핑거가고착되는 경우와 핑거의 예압이 풀리는 경우에 대한 해결 방법을 기술합니다.

그리퍼의 문제가 해소된 뒤에는 로봇 시스템에서 그리퍼의 문제를 발생 시킨 요인을분석하여 해소해야만 문제가 재발되지 않습니다.

# 7.1 Manual Release fo Stuck Finger

모터가 발생할 수 있는 최대 힘 이상의 힘에 의해 핑거의 고착된 경우 통신을통해서는 Finger를 움직일 수 없습니다. 정상적인 경우는 발생되지 않지만, 환경적인요인에 의해 정격 사양 이상의 파지력이 발생된 경우에 고착될 수 있습니다. 손으로고착된 Finger를 풀기 위해서는 그리퍼 모터 커버를 열고 손으로 모터를 돌려 주어야합니다.

아래 절차에 의해 모터를 손으로 돌릴 수 있습니다.

![](images/c538eacdb7856d2b3cfd44da93f627983f7a0db5753c3a415e4185bea2daf69c.jpg)  
Fig. 7.1: Sequence of Release Fixed Finger

Optimum 종류에 따라 모터 회전 방향에 따라 Finger의 움직임은 다릅니다. 제품별로Finger의 움직임은 아래 표에 의해 결정됩니다.

<table><tr><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>Optimum R</td><td rowspan=1 colspan=1>Optimum A</td><td rowspan=1 colspan=1>Optimum P</td></tr><tr><td rowspan=1 colspan=1>Grasp</td><td rowspan=1 colspan=1>CW</td><td rowspan=1 colspan=1>CCW</td><td rowspan=1 colspan=1>CCW</td></tr><tr><td rowspan=1 colspan=1>Release</td><td rowspan=1 colspan=1>CCW</td><td rowspan=1 colspan=1>CW</td><td rowspan=1 colspan=1>CW</td></tr></table>

Table 7.1: Rotation Direction to Release Fixed Finger according to Products

# Caution

모터 커버가 열려진 상태에서는 모터와 Palm 사이 공간에 이물질 침투시 끼임 및손가락 끼임이 발생 가능하오니 사용시 주의가 필요합니다.

# Caution

반복적인 고착이 발생된다면, Grip Torque나 Limit of Speed를 줄여서 파지력을감소시켜야 합니다.

# 7.2 Adjust Preload of Finger

Finger가 이동 방향외에 헐거워진 경우 아래에 절차에 의해 Finger의 예압을 가할 수있습니다.

필요한 도구는 육각 렌치 2mm, 3mm가 필요하며, Palm Cover Bolt의 경우 2mm렌치를, Finger 예압 조절을 위해서는 3mm 렌치를 사용합니다. 풀고 조이는 방향의정보는 아래 그림을 참조합니다.

Palm Cover Bolt는 완전히 풀지 않고 1회전 정도면 Finger의 예압을 조정하는데충분합니다. Palm Cover Bolt는 4개 모두 1회전씩 풀어 줍니다.

예압은 너무 과도한 부하를 주게되면 그리퍼 동작에 영향을 주므로 Finger의헐거워짐이 해소된 정도의 부하를 주시기 바랍니다. Finger 2개 Preload AdjustHoles에서 예압 조절이 필요한 Finger만 조절하시면 됩니다.

예압 조절 후에는 역순에 따라 Palm Cover Bolt를 고정해 주어야 합니다.

![](images/ef374bc267b4863d262b34b44f8faab010c286c7a69aa78e557e1a7e4626040b.jpg)  
Fig. 7.2: Sequence of Adjust Preload of Finger

Optimum A는 Finger End의 예압 조절이 가능한 구조이다. Palm 측 예압 조절 후에도공차가 발생된다면 아래 그림에서와 같이 Finger End 측의 예압을 조절한다.

![](images/5de9bf49c29a663469d7ec2d315e3e9678fed2fa8a082b6a7340b0da2fffaa76.jpg)  
Fig. 7.3: Adjust Preload of Optimum A Finger End