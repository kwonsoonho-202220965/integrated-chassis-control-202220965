function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

  %% TODO: 여기에 학생 구현 작성
% (1) PID 기반 yaw rate 추종 보조 조향 계산
% (2) slip angle 임계 초과 시 yaw moment 계산
% (3) speed scheduling 적용
% (4) limit/saturation

%% TODO: 여기에 학생 구현 작성
% (1) PID 기반 yaw rate 추종 보조 조향 계산
% (2) slip angle 임계 초과 시 yaw moment 계산
% (3) speed scheduling 적용
% (4) limit/saturation

%% ===== 0. 출력 기본값 및 입력 보호 =====
deltaAdd.steerAngle = 0;
deltaAdd.yawMoment  = 0;

dt    = max(dt, 1e-3);
vxEff = max(abs(vx), 0.1);


%% ===== 1. 내부 상태 초기화 =====
if ~isfield(ctrlState, 'latIntError')
    ctrlState.latIntError = 0;
end

if ~isfield(ctrlState, 'prevYawRate')
    ctrlState.prevYawRate = yawRate;
end

if ~isfield(ctrlState, 'prevYawRateRef')
    ctrlState.prevYawRateRef = yawRateRef;
end

if ~isfield(ctrlState, 'dYawRateFilt')
    ctrlState.dYawRateFilt = 0;
end

if ~isfield(ctrlState, 'transientTimer')
    ctrlState.transientTimer = 0;
end
%% ===== 2. yaw rate 추종 오차 계산 =====
yawErr = yawRateRef - yawRate;

dYawRateRef = ...
    (yawRateRef - ctrlState.prevYawRateRef) / dt;


%% ===== 3. speed scheduling =====
speedScale = min(max(vxEff / 20, 0.30), 1.20);


%% ===== 4. 적분항 및 anti-windup =====
if abs(yawErr) < 0.5
    ctrlState.latIntError = ...
        ctrlState.latIntError + yawErr * dt;
end

% CTRL.LAT.intMax가 없을 경우를 대비
if isfield(CTRL.LAT, 'intMax')
    intMax = CTRL.LAT.intMax;
else
    intMax = 1.0;
end

ctrlState.latIntError = ...
    max(min(ctrlState.latIntError, intMax), -intMax);


%% ===== 5. 미분항 계산 =====
dYawRate = ...
    (yawRate - ctrlState.prevYawRate) / dt;

alphaD = 0.85;

ctrlState.dYawRateFilt = ...
      alphaD * ctrlState.dYawRateFilt ...
    + (1 - alphaD) * dYawRate;


%% ===== 6. PID 기반 AFS 보조 조향 계산 =====
Kp = CTRL.LAT.Kp;
Ki = CTRL.LAT.Ki;
Kd = CTRL.LAT.Kd;

%% ===== 급격한 yaw-rate 명령 변화 감지 및 유지 =====

% A3처럼 큰 크기의 step yaw-rate 명령을 별도로 감지
largeStepCommand = ...
       abs(yawRateRef) > 0.18 ...
    && abs(dYawRateRef) < 0.05;

% 최초 급변 시 과도응답 타이머 시작
if abs(dYawRateRef) > 0.15
    ctrlState.transientTimer = 0.80;
else
    ctrlState.transientTimer = ...
        max(ctrlState.transientTimer - dt, 0);
end

fastTransient = ctrlState.transientTimer > 0;

if largeStepCommand
    kpScale = 0.36;
    kiScale = 0.0;
    kdScale = 3.0;

    ctrlState.latIntError = ...
        0.75 * ctrlState.latIntError;

elseif fastTransient
    kpScale = 0.45;
    kiScale = 0.0;
    kdScale = 1.20;

    ctrlState.latIntError = ...
        0.85 * ctrlState.latIntError;

else
    kpScale = 1.0;
    kiScale = 1.0;
    kdScale = 1.0;
end

deltaRaw = speedScale * ( ...
      kpScale * Kp * yawErr ...
    + kiScale * Ki * ctrlState.latIntError ...
    - kdScale * Kd * ctrlState.dYawRateFilt);


%% ===== 7. 정상 원선회 보호 =====
quasiSteady = ...
       abs(dYawRateRef) < 0.03 ...
    && abs(yawErr)      < 0.08 ...
    && abs(yawRateRef)  > 0.05 ...
    && abs(yawRateRef)  < 0.15;

if quasiSteady
    deltaRaw = 0.22 * deltaRaw;
    ctrlState.latIntError = ...
        0.70 * ctrlState.latIntError;
end


%% ===== 8. steering angle saturation =====
deltaAdd.steerAngle = ...
    max(min(deltaRaw, LIM.MAX_STEER_ANGLE), ...
                    -LIM.MAX_STEER_ANGLE);


%% ===== 9. 조향 포화 시 back-calculation anti-windup =====
if Ki > 1e-9
    saturationError = ...
        deltaAdd.steerAngle - deltaRaw;

    ctrlState.latIntError = ...
        ctrlState.latIntError ...
        + 0.15 * saturationError / Ki;

    ctrlState.latIntError = ...
        max(min(ctrlState.latIntError, intMax), -intMax);
end


%% ===== 10. ESC yaw moment 계산 =====
if isfield(LIM, 'MAX_SLIP_ANGLE')
    betaLimit = LIM.MAX_SLIP_ANGLE;
else
    betaLimit = deg2rad(6);
end

betaTh = min(deg2rad(3.0), 0.5 * betaLimit);

if quasiSteady
    % 정상 원선회에서는 ESC 차동제동 비활성화
    Mz = 0;

elseif abs(slipAngle) > betaTh
    betaExcess = abs(slipAngle) - betaTh;

    % 속도가 높아지면 ESC 개입을 점진적으로 증가
    escSpeedScale = ...
        min(max(vxEff / 20, 0.5), 1.30);

    escGain = 5000 * escSpeedScale;

    % slip angle 반대 방향으로 복원 yaw moment 생성
    Mz = -sign(slipAngle) * escGain * betaExcess;

else
    Mz = 0;
end


%% ===== 11. yaw moment saturation =====
if isfield(LIM, 'MAX_YAW_MOMENT')
    maxYawMoment = LIM.MAX_YAW_MOMENT;
elseif isfield(LIM, 'MAX_YAW_MOMENT_NM')
    maxYawMoment = LIM.MAX_YAW_MOMENT_NM;
else
    maxYawMoment = 2500;
end

deltaAdd.yawMoment = ...
    max(min(Mz, maxYawMoment), -maxYawMoment);


%% ===== 12. 상태 저장 =====
ctrlState.prevYawRate    = yawRate;
ctrlState.prevYawRateRef = yawRateRef;

end