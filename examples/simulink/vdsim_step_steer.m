% vdsim_step_steer.m
% VDSim L2 FMU를 MATLAB에서 직접 구동하는 step-steer 예제.
% Simulink 없이 MATLAB Engine으로 FMU를 step하는 가장 단순한 패턴.
%
% 사전 조건:
%   1. FMU 빌드: bash fmi_export/build_fmu.sh
%   2. Python FMU master: pip install fmpy  (또는 아래 ctypes 경로 사용)
%   3. MATLAB에서 실행: cd <VDSim_repo>; run('examples/simulink/vdsim_step_steer.m')
%
% 출력: vx, vy, yaw_rate, ay 시계열 + figure

clear; clc;

REPO = fileparts(fileparts(fileparts(mfilename('fullpath'))));
fmu_path = fullfile(REPO, 'build', 'fmi_export', 'vdsim_l2.fmu');

if ~exist(fmu_path, 'file')
    error('FMU not found: %s\nRun: bash fmi_export/build_fmu.sh', fmu_path);
end

%% -- Option A: fmpy (Python, recommended) ---------------------------------
% fmpy는 FMI 2.0/3.0을 모두 지원하는 표준 Python 라이브러리.
% MATLAB의 pyrun으로 Python 호출.

if pyenv().Version ~= ""
    % Python 경로 설정
    pyrun("import sys; sys.path.insert(0, r'" + fullfile(REPO,'build','python') + "')")
    pyrun("import sys; sys.path.insert(0, r'" + fullfile(REPO,'python') + "')")

    dt   = 0.005;    % [s]
    T    = 8.0;      % [s]
    t0_steer = 2.0;  % step-steer 시작 [s]
    steer_amp = 0.03; % [rad] wheel steer
    v_target  = 22.2; % [m/s] ~80 km/h

    N = round(T / dt);
    t   = zeros(1, N);
    vx  = zeros(1, N);
    vy  = zeros(1, N);
    r   = zeros(1, N);
    ay  = zeros(1, N);

    code = sprintf([ ...
        "from fmpy import read_model_description, extract\n" ...
        "from fmpy.simulation import simulate_fmu\n" ...
        "import numpy as np\n" ...
        "res = simulate_fmu(\n" ...
        "    '%s',\n" ...
        "    start_time=0.0, stop_time=%.3f, step_size=%.5f,\n" ...
        "    input=np.array([(t, %.3f if t>=%.2f else 0.0, max(0,min(1,(%.2f-vx[k])/3+0.05)), 0.0)\n" ...
        "                    for k,t in enumerate(np.arange(0,%.3f,%.5f))],\n" ...
        "                   dtype=[('time','f8'),('steer_angle_wheel','f8'),('throttle','f8'),('brake','f8')]),\n" ...
        "    output=['vx','vy','yaw_rate','ay_body'])\n" ...
        "vdsim_res = res\n"], ...
        strrep(fmu_path,'\','/'), T, dt, steer_amp, t0_steer, v_target, T, dt);

    try
        pyrun(code);
        res = pyrun("vdsim_res", "vdsim_res");
        t  = double(res{'time'});
        vx = double(res{'vx'});
        vy = double(res{'vy'});
        r  = double(res{'yaw_rate'});
        ay = double(res{'ay_body'});
        use_fmpy = true;
    catch ME
        warning('fmpy failed (%s). Falling back to manual FMU stepping.', ME.message);
        use_fmpy = false;
    end
end

%% -- Option B: manual Python FMUMaster (fallback) -------------------------
if ~exist('use_fmpy','var') || ~use_fmpy
    % VDSim 동봉 FMUMaster(ctypes 기반) 사용
    pyrun("import sys; sys.path.insert(0, r'" + fullfile(REPO,'fmi_export') + "')")
    pyrun("from fmu_master import FMUMaster")
    pyrun(sprintf("fmu = FMUMaster.load('%s')", strrep(fmu_path,'\','/')));
    pyrun("fmu.initialize(0.0)");

    dt = 0.005; T = 8.0; N = round(T/dt);
    t  = (0:N-1)*dt;
    vx = zeros(1,N); vy = zeros(1,N); r = zeros(1,N); ay = zeros(1,N);

    for k = 1:N
        tk = (k-1)*dt;
        steer   = 0.03 * (tk >= 2.0);
        cur_vx  = vx(max(1,k-1));
        throttle = max(0, min(1, (22.2 - cur_vx)/3 + 0.05));
        pyrun(sprintf("fmu.set('steer_angle_wheel', %.5f)", steer));
        pyrun(sprintf("fmu.set('throttle', %.5f)", throttle));
        pyrun(sprintf("fmu.do_step(%.5f, %.5f)", tk, dt));
        st = pyrun("s = fmu.get_many('vx','vy','yaw_rate','ay_body')", "s");
        vx(k) = double(st{'vx'});
        vy(k) = double(st{'vy'});
        r(k)  = double(st{'yaw_rate'});
        ay(k) = double(st{'ay_body'});
    end
    pyrun("fmu.free()");
end

%% -- Plot -----------------------------------------------------------------
figure('Name','VDSim L2 — Step Steer');
subplot(2,2,1); plot(t, vx); xlabel('t [s]'); ylabel('vx [m/s]'); title('Longitudinal speed'); grid on;
subplot(2,2,2); plot(t, ay); xlabel('t [s]'); ylabel('ay [m/s^2]'); title('Lateral accel'); grid on;
subplot(2,2,3); plot(t, r);  xlabel('t [s]'); ylabel('r [rad/s]'); title('Yaw rate'); grid on;
subplot(2,2,4); plot(t, vy); xlabel('t [s]'); ylabel('vy [m/s]'); title('Lateral speed'); grid on;
sgtitle('VDSim FMU — Step Steer (80 km/h, \delta=0.03 rad)');

fprintf('Peak ay = %.3f m/s^2\n', max(abs(ay)));
fprintf('Peak r  = %.4f rad/s\n', max(abs(r)));
