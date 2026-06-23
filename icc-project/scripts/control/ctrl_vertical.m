function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

    %% TODO: 학생 구현
    %  (1) skyhook (또는 변형)
    %  (2) per-wheel 적용
    %  (3) cMin/cMax 제한

   %% Vertical controller implementation
% Skyhook 기반 semi-active damping 제어를 구현한다.
% sprung mass 속도와 suspension relative velocity의 방향을 이용해
% 각 wheel별 damping coefficient를 CMin/CMax 사이에서 선택한다.

% ===== 1. damping limit 설정 =====
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

if isfield(CTRL.VER, 'skyGain')
    cSky = CTRL.VER.skyGain;
elseif isfield(CTRL.VER, 'SkyGain')
    cSky = CTRL.VER.SkyGain;
else
    cSky = 2500;
end

% ===== 2. suspension state 읽기 =====
% 기본값
zs_dot = zeros(4, 1);
zu_dot = zeros(4, 1);

if isfield(suspState, 'zs_dot')
    zs_dot = suspState.zs_dot(:);
end

if isfield(suspState, 'zu_dot')
    zu_dot = suspState.zu_dot(:);
end

% 4x1 형태 보장
if numel(zs_dot) < 4
    zs_dot = zeros(4, 1);
else
    zs_dot = zs_dot(1:4);
end

if numel(zu_dot) < 4
    zu_dot = zeros(4, 1);
else
    zu_dot = zu_dot(1:4);
end

% ===== 3. Skyhook semi-active logic =====
relVel = zs_dot - zu_dot;

dampingCmd = zeros(4, 1);

for i = 1:4
    % Skyhook 조건:
    % sprung velocity와 relative velocity의 곱이 양수이면 감쇠를 크게,
    % 아니면 감쇠를 작게 설정한다.
    if zs_dot(i) * relVel(i) > 0
        dampingCmd(i) = cMax;
    else
        dampingCmd(i) = cMin;
    end
end

% ===== 4. 급격한 변화 완화 =====
% 너무 딱딱한 on/off를 피하기 위해 skyGain 기준으로 한 번 완화
dampingCmd = 0.7 * dampingCmd + 0.3 * cSky * ones(4, 1);

% ===== 5. 최종 saturation =====
dampingCmd = max(min(dampingCmd, cMax), cMin);;

end
