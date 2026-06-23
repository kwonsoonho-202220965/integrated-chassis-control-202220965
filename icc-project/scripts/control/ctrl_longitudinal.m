function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% TODO: 여기에 학생 구현
    %  (1) speed-tracking PI
    %  (2) ABS modulation (이번 함수에서 또는 ctrl_coordinator 에서)
    %  (3) jerk limit
    %  (4) anti-windup

    % 임시 baseline (반드시 본인 설계로 교체)
    %% Longitudinal controller implementation
    % 속도 추종 PI 제어와 간단한 ABS slip limiting을 구현하였다.

    % ===== 1. 내부 상태 초기화 =====
    if ~isfield(ctrlState, 'lonIntError')
        ctrlState.lonIntError = 0;
    end

    if ~isfield(ctrlState, 'prevFxTotal')
        ctrlState.prevFxTotal = 0;
    end

    % ===== 2. 속도 오차 계산 ====
    vErr = vxRef - vx;

    % ===== 3. PI 기반 목표 종방향 가속도 계산 =====
    ctrlState.lonIntError = ctrlState.lonIntError + vErr * dt;

    
    ctrlState.lonIntError = max(min(ctrlState.lonIntError, CTRL.LON.intMax), ...
        -CTRL.LON.intMax);

    axCmd = CTRL.LON.Kp * vErr + CTRL.LON.Ki * ctrlState.lonIntError;

    
    axCmd = max(min(axCmd, LIM.MAX_AX), -LIM.MAX_AX);

    % ===== 4. 종방향 힘 요구값 계산 =====
    mVeh = 1500;
    FxDesired = mVeh * axCmd;

    % ===== 5. jerk limit / force rate limit =====
    dFxMax = mVeh * LIM.MAX_JERK * dt;
    FxDesired = max(min(FxDesired, ctrlState.prevFxTotal + dFxMax), ...
        ctrlState.prevFxTotal - dFxMax);

%% ===== 6. brakeRatio 계산 =====
strongDecelRequest = ...
       vErr < -2.0 ...
    && axCmd < -0.5;

if strongDecelRequest
    forceCmd.brakeRatio = ...
        min(max(-axCmd / max(LIM.MAX_AX, 0.1), 0.0), 1.0);
else
    forceCmd.brakeRatio = 0.0;
end

    % ===== 7. 간단한 ABS slip limiting =====
slipAbsMax = 0;

if isfield(ctrlState, 'wheelSlip')
    slipAbsMax = max(abs(ctrlState.wheelSlip(:)));
elseif isfield(ctrlState, 'slipRatio')
    slipAbsMax = max(abs(ctrlState.slipRatio(:)));
elseif isfield(ctrlState, 'wheelSlipRatio')
    slipAbsMax = max(abs(ctrlState.wheelSlipRatio(:)));
end

% slip 정보가 있으면 ABS처럼 제동비를 낮춤
if slipAbsMax > 0.12 && forceCmd.brakeRatio > 0
    reduction = max(0.35, 1.0 - 2.0 * (slipAbsMax - 0.12));
    forceCmd.brakeRatio = forceCmd.brakeRatio * reduction;
end

    % ===== 8. 최종 saturation =====
    forceCmd.brakeRatio = max(min(forceCmd.brakeRatio, 1.0), 0.0);

    % brakeRatio와 Fx_total의 방향 일관성 유지
    if forceCmd.brakeRatio > 0
        forceCmd.Fx_total = -abs(FxDesired);
    else
        forceCmd.Fx_total = FxDesired;
    end

    % 전체 종방향 힘 제한
    FxLimit = mVeh * LIM.MAX_AX;
    forceCmd.Fx_total = max(min(forceCmd.Fx_total, FxLimit), -FxLimit);

    % ===== 9. 이전 force 저장 =====
    ctrlState.prevFxTotal = forceCmd.Fx_total;
end
