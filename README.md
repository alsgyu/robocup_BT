## Nomadz Behavior

Nomadz팀의 로보컵 축구 환경에서 로봇의 행동 결정을 관리하는 Behavior Tree (BT) 기반의 ROS 패키지다.

이 패키지는 행동 제어 로직(BehaviorTree.xml)과 머리 움직임(HeadMotionTree.xml)을 분리하여 병렬 제어를 수행하며, 
각 노드는 YAML 설정 및 ROS 인터페이스를 통해 상호작용한다.
</br>

### config/

---

> 로봇의 행동 트리 및 설정 파일들이 포함된다.

#### 1. BehaviorTree.xml – 메인 행동 트리 정의 (BehaviorMainTree)

경기 상태(game_state, game_phase) 및 역할(player_role)에 따라 트리 전환 수행

주요 서브트리: Game_Play_ST, Engage_ST, Keeper_Behavior_ST, Alignment_ST

</br>

#### 2. HeadMotionTree.xml – 시각 탐색 및 머리 움직임 제어 트리

LookAtBall, LookAtDirection 등으로 시야 스캔 수행

</br>

#### 3. TreeNodesModel.xml – Groot에서 사용되는 노드 모델 설명
</br>

#### 4. behavior.yaml – 주요 파라미터 정의

main_tree_name: "BehaviorMainTree"</br>
head_motion_tree_name: "HeadMotionTree"</br>
bt_loop_frequency: 30.0</br>
has_ball_distance: 0.35</br>

</br>

#### 5. target_pose_generator.yaml – 포지션 및 거리 최적화 파라미터

공격/수비 위치, 공과 거리, 목표 탐색 비용 함수 설정 포함

target_pose_generator

</br>

### include/nomadz_behavior/
---


> C++ 헤더 구조로, 각 기능별 노드 정의를 포함한다.

#### action_nodes/

high_level_actions.hpp: Alignment, Engage, Kick 등 상위 행동 정의

...



### src/
---

> 실제 동작 구현

#### 1. action_nodes/ – 각 행동 노드 구현 (WalkToTarget, SaveShot, Kick, 등)

#### 2. check_nodes/ – 조건 판단 노드 구현 (BallFound, Penalized, 등)

#### 3. debug_nodes/ – 디버깅용 노드

#### 4. helper/ – 내부 보조 기능 및 유틸리티

#### 5. main.cpp – BT 실행 엔트리포인트

#### 6. ros_interface.cpp – ROS 메시지 통신 및 topic/pub 관리

</br>

### nomadz_behavior_msgs/
---

> BT 동작에 필요한 메시지 타입(SpecialActionType, Pose2D, Twist2D)을 정의한다.
