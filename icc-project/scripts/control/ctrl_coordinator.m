function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    %  (1) lonCmd.Fx_total → 4-wheel 균등 brake (with 60:40 split)
    %  (2) latCmd.yawMoment → 4-wheel 차동 brake
    %  (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
    %  (4) verCmd → actuatorCmd.dampingCoeff (pass-through 또는 추가 가공)
    %  (5) 최종 saturation

    % 임시 baseline (반드시 교체)
   %% Coordinator implementation
% lateral / longitudinal / vertical controller 명령을 실제 actuator 명령으로 변환한다.
% - AFS steer angle saturation
% - longitudinal brakeRatio -> 4-wheel brake torque
% - ESC yaw moment -> left/right differential brake torque
% - vertical damping command pass-through with saturation

% ===== 1. AFS 조향각 제한 =====
actuatorCmd.steerAngle = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), ...
                            -LIM.MAX_STEER_ANGLE);

% ===== 2. 기본 4-wheel brake torque 계산 =====
brakeRatio = max(min(lonCmd.brakeRatio, 1.0), 0.0);

if isfield(LIM, 'MAX_BRAKE_TQ')
    maxBrakeTq = LIM.MAX_BRAKE_TQ;
elseif isfield(LIM, 'MAX_BRAKE_TORQUE')
    maxBrakeTq = LIM.MAX_BRAKE_TORQUE;
else
    maxBrakeTq = 3000;
end

% ===== straight braking fallback =====
straightLike = abs(latCmd.steerAngle) < deg2rad(0.08) ...
    && abs(latCmd.yawMoment) < 5;

if straightLike && vx > 13
    speedRatio = min(max((vx - 13) / 15, 0.0), 1.0);

    minBrakeRatio = 0.30 + 0.70 * speedRatio;

    brakeRatio = max(brakeRatio, minBrakeRatio);
end

brakeRatio = max(min(brakeRatio, 1.0), 0.0);

% 기본 제동 토크
baseBrakeTq = brakeRatio * maxBrakeTq;

% 전륜/후륜 분배
% 순서: [FL; FR; RL; RR]
frontBias = 0.60;
rearBias  = 0.40;

brakeTorque = zeros(4, 1);
brakeTorque(1) = baseBrakeTq * frontBias / 2;
brakeTorque(2) = baseBrakeTq * frontBias / 2;
brakeTorque(3) = baseBrakeTq * rearBias / 2;
brakeTorque(4) = baseBrakeTq * rearBias / 2;

% ===== 3. ESC yaw moment를 좌우 차동 제동으로 변환 =====
Mz = latCmd.yawMoment;

track = max(VEH.track_f, 0.1);
rw = max(VEH.rw, 0.1);

dT = abs(Mz) * rw / track;

dT = min(dT, 0.5 * maxBrakeTq);

if Mz > 0
    brakeTorque(1) = brakeTorque(1) + dT;
    brakeTorque(3) = brakeTorque(3) + 0.7 * dT;
elseif Mz < 0
    brakeTorque(2) = brakeTorque(2) + dT;
    brakeTorque(4) = brakeTorque(4) + 0.7 * dT;
end

% ===== 4. brake torque saturation =====
if straightLike && vx > 13
  
    speedRatio = min(max((vx - 13) / 15, 0.0), 1.0);
    minBrakeTq = 1000 + 900 * speedRatio;

    brakeTorque = ...
        max(brakeTorque, minBrakeTq * ones(4, 1));
end

actuatorCmd.brakeTorque = ...
    max(min(brakeTorque, maxBrakeTq), 0.0);


% ===== 5. damping coefficient 전달 및 제한 =====
if isfield(CTRL.VER, 'CMin')
    cMin = CTRL.VER.CMin;
elseif isfield(CTRL.VER, 'Cmin')
    cMin = CTRL.VER.Cmin;
else
    cMin = 500;
end

if isfield(CTRL.VER, 'CMax')
    cMax = CTRL.VER.CMax;
elseif isfield(CTRL.VER, 'Cmax')
    cMax = CTRL.VER.Cmax;
else
    cMax = 5000;
end

if isempty(verCmd)
    actuatorCmd.dampingCoeff = cMin * ones(4, 1);
else
    actuatorCmd.dampingCoeff = verCmd(:);
end

% 4x1 형태 보장
if numel(actuatorCmd.dampingCoeff) == 1
    actuatorCmd.dampingCoeff = actuatorCmd.dampingCoeff * ones(4, 1);
end

% 4개보다 적게 들어오는 경우 방어
if numel(actuatorCmd.dampingCoeff) < 4
    actuatorCmd.dampingCoeff = cMin * ones(4, 1);
else
    actuatorCmd.dampingCoeff = actuatorCmd.dampingCoeff(1:4);
end

% damping saturation
actuatorCmd.dampingCoeff = max(min(actuatorCmd.dampingCoeff, cMax), cMin);
end
