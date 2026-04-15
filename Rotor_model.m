clc;
clear;

% ROTOR_SINGLE_COMPLETE_DEMO
% Self-contained single-rotor BEMT + flap trim solver.
% 
% In this version, you EXPLICITLY choose the model source in the rotor struct:
%   rotor.airfoil_model  = 'linear'  or 'c81txt'
%   rotor.chord_model    = 'linear'  or 'lookup'
%   rotor.pretwist_model = 'linear'  or 'lookup'
%
% Rules:
%   1) If you choose lookup/C81 and the required file is missing -> ERROR.
%   2) Defaults are used only when you explicitly choose 'linear'.
%   3) This is the behaviour you asked for.

%% Example state
state = struct();
state.velo_body = [0; 0; 0];
state.angular_velocity_body = [0; 0; 0];
state.acceleration_body = [0; 0; 0];
state.angular_acceleration_body = [0; 0; 0];

%% Rotor definition
rotor = struct();
rotor.R = 1.4;
rotor.Nb = 4;
rotor.omega = 90;
rotor.rho = 1.225;
rotor.I_beta = 2;
rotor.k_beta = 15000;
rotor.tilt_angle_deg = 90.0;
rotor.rotational_direction = 1;      % +1 or -1
rotor.hub_loc = [0; 0; 0];           % body axes, relative to CG

rotor.root_cutout = 0.1;
rotor.n_be = 10;
rotor.n_az = 72;

%% EXPLICIT model selection
% airfoil_model : 'linear' or 'c81txt'
% chord_model   : 'linear' or 'lookup'
% pretwist_model: 'linear' or 'lookup'
rotor.airfoil_model = 'linear';
rotor.chord_model = 'linear';
rotor.pretwist_model = 'linear';

%% Spanwise sectioning
% User controls the section boundaries and which airfoil each section uses.
rotor.section_r_end = [1 1 1 1 1 1];
rotor.section_airfoil_id = [1 1 1 1 1 1];

%% AIRFOIL configuration
switch lower(rotor.airfoil_model)
    case 'linear'
        % Explicit default linear model
        rotor.airfoils(1).type = 'linear';
        rotor.airfoils(1).a0 = 5.7;          % per rad
        rotor.airfoils(1).alpha0_deg = 0.0;
        rotor.airfoils(1).cd0 = 0.01;
        rotor.airfoils(1).k = 0.02;

    case 'c81txt'
        % Fill the pairs you really want to use.
        % If a required file is missing, the code will STOP with an error.
        c81_pairs(1).cl_file = 'CS1_cl.txt'; c81_pairs(1).cd_file = 'CS1_cd.txt';
        c81_pairs(2).cl_file = 'CS2_cl.txt'; c81_pairs(2).cd_file = 'CS2_cd.txt';
        c81_pairs(3).cl_file = 'CS3_cl.txt'; c81_pairs(3).cd_file = 'CS3_cd.txt';
        c81_pairs(4).cl_file = 'CS4_cl.txt'; c81_pairs(4).cd_file = 'CS4_cd.txt';
        c81_pairs(5).cl_file = 'CS5_cl.txt'; c81_pairs(5).cd_file = 'CS5_cd.txt';
        c81_pairs(6).cl_file = 'CS6_cl.txt'; c81_pairs(6).cd_file = 'CS6_cd.txt';
        rotor = configure_airfoils_from_c81txt(rotor, c81_pairs);

    otherwise
        error('rotor.airfoil_model must be ''linear'' or ''c81txt''.');
end

%% CHORD configuration
switch lower(rotor.chord_model)
    case 'linear'
        % Explicit default linear chord
        rotor.chord.type = 'linear';
        rotor.chord.unit = 'meter';
        rotor.chord.root_value = 0.18;
        rotor.chord.tip_value  = 0.18;

    case 'lookup'
        % Require lookup data to exist; otherwise STOP.
        rotor.chord.lookup_mat = 'chord_interp.mat';
        rotor.chord.lookup_var = 'F';
        rotor.chord.lookup_unit = 'c_over_R';
        rotor = configure_chord_lookup(rotor);

    otherwise
        error('rotor.chord_model must be ''linear'' or ''lookup''.');
end

%% PRETWIST configuration
switch lower(rotor.pretwist_model)
    case 'linear'
        % Explicit default linear pretwist (deg)
        rotor.pretwist.type = 'linear';
        rotor.pretwist.root_deg = 0.0;
        rotor.pretwist.tip_deg  = -12.0;

    case 'lookup'
        % Require lookup data to exist; otherwise STOP.
        rotor.pretwist.lookup_mat = 'pretwist_interp.mat';
        rotor.pretwist.lookup_var = 'pre_twist';
        rotor = configure_pretwist_lookup(rotor);

    otherwise
        error('rotor.pretwist_model must be ''linear'' or ''lookup''.');
end

%% Control input
theta0_deg = 15;

%% Solver options
options = struct();
options.max_iter = 40;
options.tol = 1e-10;
options.fd_step = 1e-4;
options.damping = 1.0;
options.min_damping = 0.15;
options.initial_guess = [zeros(2*rotor.Nb,1); 1.0];

%% Solve
[forces, info] = rotor_bemt_trim(rotor, state, theta0_deg, options);

%% Display
fprintf('\n=== Single Rotor Result (Complete Demo) ===\n');
fprintf('Converged      : %d\n', info.converged);
fprintf('Iterations     : %d\n', info.iter);
fprintf('Residual norm  : %.6e\n', info.residual_norm);
fprintf('Power (W)      : %.6f\n', info.power);
fprintf('Induced vi (m/s): %.6f\n', info.trim.vi);
fprintf('Forces/Moments [Fx Fy Fz Mx My Mz]^T:\n');
disp(forces);

fprintf('Initial flap angles beta0 (rad):\n');
disp(info.trim.beta0.');
fprintf('Initial flap rates dbeta0 (rad/s):\n');
disp(info.trim.dbeta0.');

function [forces, info] = rotor_bemt_trim(rotor, state, theta0_deg, options)
% Single-rotor BEMT + flap trim solved internally.
% Output:
%   forces = [Fx; Fy; Fz; Mx; My; Mz] in body axes

    if nargin < 4
        options = struct();
    end

    options = fill_default_options(options, rotor);
    x = options.initial_guess(:);   % [beta0(1:Nb); dbeta0(1:Nb); vi]

    converged = false;

    for iter = 1:options.max_iter
        [residual, forces, aux] = rotor_trim_residual(x, rotor, state, theta0_deg);
        resnorm = norm(residual);

        if resnorm < options.tol
            converged = true;
            break;
        end

        n = numel(x);
        J = zeros(n, n);
        for i = 1:n
            dx = options.fd_step * max(1.0, abs(x(i)));
            xp = x;
            xp(i) = xp(i) + dx;
            rp = rotor_trim_residual(xp, rotor, state, theta0_deg);
            J(:, i) = (rp - residual) / dx;
        end

        step = -J \ residual;

        alpha = options.damping;
        accepted = false;
        for ls = 1:8
            x_try = x + alpha * step;
            r_try = rotor_trim_residual(x_try, rotor, state, theta0_deg);
            if norm(r_try) < resnorm
                x = x_try;
                accepted = true;
                break;
            end
            alpha = alpha * 0.5;
        end

        if ~accepted
            x = x + options.min_damping * step;
        end
    end

    [residual, forces, aux] = rotor_trim_residual(x, rotor, state, theta0_deg);

    info = struct();
    info.trim.beta0 = x(1:rotor.Nb);
    info.trim.dbeta0 = x(rotor.Nb+1:2*rotor.Nb);
    info.trim.vi = x(end);
    info.power = aux.power;
    info.converged = converged;
    info.residual = residual;
    info.residual_norm = norm(residual);
    info.iter = iter;
    info.beta_hist = aux.beta_hist;
    info.dbeta_hist = aux.dbeta_hist;
    info.alpha_hist = aux.alpha_hist;
end


function [residual, forces, aux] = rotor_trim_residual(x, rotor, state, theta0_deg)
    Nb = rotor.Nb;

    beta0 = x(1:Nb);
    dbeta0 = x(Nb+1:2*Nb);
    vi = x(end);

    [forces, beta_hist, dbeta_hist, power, alpha_hist, flow_state] = ...
        rotor_forces_one_rev(beta0, dbeta0, vi, rotor, state, theta0_deg);

    residual = zeros(2*Nb + 1, 1);
    residual(1:Nb) = beta_hist(end, :).'-beta0(:);
    residual(Nb+1:2*Nb) = dbeta_hist(end, :).'-dbeta0(:);

    residual(end) = ( ...
        flow_state.Tb_average + ...
        2*rotor.rho*pi*rotor.R^2 * ...
        sqrt((flow_state.V_z + vi)^2 + flow_state.V_inf^2 + flow_state.V_infy^2) * vi ...
        ) / (rotor.rho*pi*rotor.R^2*rotor.omega^2);

    aux = struct();
    aux.power = power;
    aux.beta_hist = beta_hist;
    aux.dbeta_hist = dbeta_hist;
    aux.alpha_hist = alpha_hist;
end


function [forces, beta_hist, dbeta_hist, power, alpha_hist, flow_state] = ...
    rotor_forces_one_rev(beta0, dbeta0, vi, rotor, state, theta0_deg)

    rho = rotor.rho;
    R = rotor.R;
    Nb = rotor.Nb;
    omega = rotor.omega * rotor.rotational_direction;
    I_beta = rotor.I_beta;
    k_beta = rotor.k_beta;

    n_be = rotor.n_be;
    n_az = rotor.n_az;
    root_cutout = rotor.root_cutout;

    dpsi = 2*pi / n_az;
    dt = dpsi / abs(omega);
    dr_phys = (R - root_cutout*R) / n_be;

    alpha_hist = zeros(n_az, n_be);

    veloo = state.velo_body(:) + cross(state.angular_velocity_body(:), rotor.hub_loc(:));

    tilt = deg2rad(rotor.tilt_angle_deg);
    T_body2disc = [ sin(tilt)  0  cos(tilt);
                    0          1  0;
                   -cos(tilt)  0  sin(tilt) ];
    T_disc2body = T_body2disc';

    v_disc = T_body2disc * veloo;
    w_disc = T_body2disc * state.angular_velocity_body(:);
    a_disc = T_body2disc * state.acceleration_body(:);
    ang_acc_disc = T_body2disc * state.angular_acceleration_body(:);

    ppp = w_disc(1);
    qqq = w_disc(2);
    uuu = v_disc(1);
    vvv = v_disc(2);
    aaw = a_disc(3);
    aap = ang_acc_disc(1);
    aaq = ang_acc_disc(2);

    theta0 = deg2rad(theta0_deg);

    V_inf  = v_disc(1);
    V_infy = v_disc(2);
    V_z    = v_disc(3);
    velocity1 = [V_inf; V_infy; V_z];

    beta_hist = zeros(n_az+1, Nb);
    dbeta_hist = zeros(n_az+1, Nb);
    beta_hist(1, :) = beta0(:).';
    dbeta_hist(1, :) = dbeta0(:).';

    Tb_average = 0;
    Hb_average = 0;
    Sb_average = 0;
    Torque_average = 0;
    M_average = 0;
    L_average = 0;

    for blade = 1:Nb
        beta_dot = dbeta_hist(1, blade);

        for iaz = 1:n_az
            Tb_new = 0;
            Hb_new = 0;
            Sb_new = 0;
            M_A = 0;
            M_torque = 0;

            Az = rotor.rotational_direction * ((iaz-1)*dpsi + (blade-1)*2*pi/Nb);

            T_az = [-cos(Az) -sin(Az)  0;
                     sin(Az) -cos(Az)  0;
                     0        0        1];
            T_az_inv = T_az';

            beta_now = beta_hist(iaz, blade);
            T_flap = [ cos(beta_now) 0 -sin(beta_now);
                       0             1  0;
                       sin(beta_now) 0  cos(beta_now) ];
            T_flap_inv = T_flap';

            velocity2 = T_az * velocity1;
            w_shaft = T_az * w_disc + [0;0;omega];
            w_blade = T_flap * w_shaft + [0; beta_dot; 0];
            velocity_blade_root = T_flap * velocity2;

            for iz = 1:n_be
                x = root_cutout + (1-root_cutout)/(2*n_be) + (iz-1)*(1-root_cutout)/n_be;
                r = x * R;

                v_blade_elem = velocity_blade_root + cross(w_blade, [r;0;0]);
                pretwist_deg = eval_pretwist(rotor, x);
                theta = theta0 + deg2rad(pretwist_deg);

                V_n = v_blade_elem(3) - vi*cos(beta_now);
                vt  = rotor.rotational_direction * v_blade_elem(2);

                phi_new = atan2(V_n, vt);
                alpha_new = theta + phi_new;

                if blade == 1
                    alpha_hist(iaz, iz) = alpha_new;
                end

                Mach = sqrt(V_n^2 + vt^2) / 340;
                [CL, CD] = eval_airfoil(rotor, x, rad2deg(alpha_new), Mach);
                chord_val = eval_chord(rotor, x);

                dL = 0.5 * rho * (vt^2 + (vi - V_n)^2) * chord_val * dr_phys * CL;
                dD = 0.5 * rho * (vt^2 + (vi - V_n)^2) * chord_val * dr_phys * CD;

                T_be = -(dL*cos(phi_new) + dD*sin(phi_new));
                D_be = -rotor.rotational_direction*dD*cos(phi_new) + ...
                        rotor.rotational_direction*dL*sin(phi_new);

                M_torque = M_torque + D_be*r;

                blade_force = [0; D_be; T_be];
                blade_force = T_flap_inv * blade_force;
                blade_force = T_az_inv * blade_force;

                Tb_new = Tb_new + blade_force(3);
                Hb_new = Hb_new + blade_force(1);
                Sb_new = Sb_new + blade_force(2);
                M_A = M_A + blade_force(3)*r;
            end

            Tb_average = Tb_average + Tb_new / n_az;
            Hb_average = Hb_average + Hb_new / n_az;
            Sb_average = Sb_average + Sb_new / n_az;
            Torque_average = Torque_average + M_torque / n_az;

            M_CF = -omega^2 * I_beta * beta_now;
            M_R  = -k_beta * beta_now;
            M_cor = -2 * I_beta * (ppp*omega*cos(Az) - qqq*omega*sin(Az));
            M_ba = I_beta * (aap*sin(Az) + aaq*cos(Az));
            M_bl = 1.5 * (aaw - uuu*qqq + ppp*vvv);

            M_average = M_average + M_R*cos(Az)/n_az;
            L_average = L_average + M_R*sin(Az)/n_az;

            beta_2dot = (M_A + M_CF + M_R + M_cor + M_ba + M_bl) / I_beta;
            beta_dot = beta_dot + beta_2dot * dt;

            dbeta_hist(iaz+1, blade) = beta_dot;
            beta_hist(iaz+1, blade) = beta_hist(iaz, blade) + beta_dot * dt;
        end
    end

    forces_disc = [Hb_average; Sb_average; Tb_average];
    forces_body = T_disc2body * forces_disc;

    moments_disc = [L_average; M_average; Torque_average];
    moments_body_from_disc = T_disc2body * moments_disc;

    aero_moment_from_hub = cross(rotor.hub_loc(:), forces_body(:));

    forces = zeros(6,1);
    forces(1:3) = forces_body;
    forces(4:6) = moments_body_from_disc + aero_moment_from_hub;

    power = abs(Torque_average) * abs(omega);

    flow_state = struct();
    flow_state.Tb_average = Tb_average;
    flow_state.V_inf = V_inf;
    flow_state.V_infy = V_infy;
    flow_state.V_z = V_z;
end


function [CL, CD] = eval_airfoil(rotor, x, alpha_deg, Mach)
    idx = find(x <= rotor.section_r_end, 1, 'first');
    if isempty(idx)
        idx = numel(rotor.section_r_end);
    end

    airfoil_id = rotor.section_airfoil_id(idx);
    af = rotor.airfoils(airfoil_id);

    switch lower(af.type)
        case {'c81','c81txt'}
            CL = af.cl(alpha_deg, Mach);
            CD = af.cd(alpha_deg, Mach);
        case 'linear'
            alpha_eff_rad = deg2rad(alpha_deg - af.alpha0_deg);
            CL = af.a0 * alpha_eff_rad;
            CD = af.cd0 + af.k * CL^2;
        otherwise
            error('Unknown airfoil type: %s', af.type);
    end
end


function chord_m = eval_chord(rotor, x)
    switch lower(rotor.chord.type)
        case 'function'
            raw = rotor.chord.fun(x);
        case 'linear'
            s = (x - rotor.root_cutout) / (1 - rotor.root_cutout);
            s = min(max(s, 0), 1);
            raw = rotor.chord.root_value + s * (rotor.chord.tip_value - rotor.chord.root_value);
        otherwise
            error('Unknown chord type: %s', rotor.chord.type);
    end

    switch lower(rotor.chord.unit)
        case 'c_over_r'
            chord_m = raw * rotor.R;
        case 'meter'
            chord_m = raw;
        otherwise
            error('Unknown chord unit: %s', rotor.chord.unit);
    end
end


function pretwist_deg = eval_pretwist(rotor, x)
    switch lower(rotor.pretwist.type)
        case 'function'
            pretwist_deg = rotor.pretwist.fun(x);
        case 'linear'
            s = (x - rotor.root_cutout) / (1 - rotor.root_cutout);
            s = min(max(s, 0), 1);
            pretwist_deg = rotor.pretwist.root_deg + s * (rotor.pretwist.tip_deg - rotor.pretwist.root_deg);
        otherwise
            error('Unknown pretwist type: %s', rotor.pretwist.type);
    end
end


function options = fill_default_options(options, rotor)
    if ~isfield(options, 'max_iter'), options.max_iter = 40; end
    if ~isfield(options, 'tol'), options.tol = 1e-10; end
    if ~isfield(options, 'fd_step'), options.fd_step = 1e-4; end
    if ~isfield(options, 'damping'), options.damping = 1.0; end
    if ~isfield(options, 'min_damping'), options.min_damping = 0.15; end
    if ~isfield(options, 'initial_guess')
        options.initial_guess = [zeros(2*rotor.Nb,1); 1.0];
    end
end


function rotor = configure_airfoils_from_c81txt(rotor, c81_pairs)
    max_id = max(rotor.section_airfoil_id);

    if numel(c81_pairs) < max_id
        error('rotor.airfoil_model = c81txt, but c81_pairs is incomplete. Need %d pairs.', max_id);
    end

    rotor.airfoils = repmat(struct('type','','cl',[],'cd',[]), 1, max_id);
    for k = 1:max_id
        if ~isfield(c81_pairs(k), 'cl_file') || ~isfield(c81_pairs(k), 'cd_file')
            error('Missing cl_file or cd_file for airfoil section %d.', k);
        end
        cl_file = c81_pairs(k).cl_file;
        cd_file = c81_pairs(k).cd_file;

        if exist(cl_file, 'file') ~= 2
            error('Selected c81txt airfoil model, but CL table file not found: %s', cl_file);
        end
        if exist(cd_file, 'file') ~= 2
            error('Selected c81txt airfoil model, but CD table file not found: %s', cd_file);
        end

        rotor.airfoils(k).type = 'c81txt';
        rotor.airfoils(k).cl = build_c81_lookup_from_txt(cl_file);
        rotor.airfoils(k).cd = build_c81_lookup_from_txt(cd_file);
    end
end


function rotor = configure_chord_lookup(rotor)
    matfile = rotor.chord.lookup_mat;
    varname = rotor.chord.lookup_var;

    if exist(matfile, 'file') ~= 2
        error('Selected chord lookup model, but mat file not found: %s', matfile);
    end

    S = load(matfile);
    if ~isfield(S, varname)
        error('Selected chord lookup model, but variable "%s" is missing in %s.', varname, matfile);
    end

    raw = S.(varname);
    rotor.chord.type = 'function';
    rotor.chord.unit = rotor.chord.lookup_unit;
    rotor.chord.fun = make_callable_1arg(raw, sprintf('chord variable "%s" in %s', varname, matfile));
end


function rotor = configure_pretwist_lookup(rotor)
    matfile = rotor.pretwist.lookup_mat;
    varname = rotor.pretwist.lookup_var;

    if exist(matfile, 'file') ~= 2
        error('Selected pretwist lookup model, but mat file not found: %s', matfile);
    end

    S = load(matfile);
    if ~isfield(S, varname)
        error('Selected pretwist lookup model, but variable "%s" is missing in %s.', varname, matfile);
    end

    raw = S.(varname);
    rotor.pretwist.type = 'function';
    rotor.pretwist.fun = make_callable_1arg(raw, sprintf('pretwist variable "%s" in %s', varname, matfile));
end


function Ffun = make_callable_1arg(raw, desc)
    if isa(raw, 'function_handle')
        Ffun = @(x) raw(x);
        return;
    end

    % Allow common interpolant objects that can be called like raw(x)
    try
        raw(0.5);
        Ffun = @(x) raw(x);
        return;
    catch
    end

    error('The %s is not callable with one input x.', desc);
end


function Ffun = build_c81_lookup_from_txt(filename)
    % Robust parser for txt tables of the form:
    % first row:    Mach1 Mach2 Mach3 ...
    % next rows:    alpha  val1  val2  val3 ...
    %
    % This matches your current C81 txt files.

    if exist(filename, 'file') ~= 2
        error('C81 txt file not found: %s', filename);
    end

    fid = fopen(filename, 'r');
    if fid < 0
        error('Cannot open C81 txt file: %s', filename);
    end

    cleaner = onCleanup(@() fclose(fid));

    lines = {};
    while ~feof(fid)
        tline = strtrim(fgetl(fid));
        if ischar(tline) && ~isempty(tline)
            lines{end+1} = tline; %#ok<AGROW>
        end
    end

    if numel(lines) < 2
        error('C81 txt file has too few lines: %s', filename);
    end

    % -------- first line: Mach grid --------
    mach_grid = sscanf(lines{1}, '%f').';
    if isempty(mach_grid)
        error('Failed to read Mach grid from first line of %s', filename);
    end

    nMach = numel(mach_grid);

    % -------- remaining lines: alpha + coefficients --------
    alpha_grid = zeros(numel(lines)-1, 1);
    coeff = zeros(numel(lines)-1, nMach);

    for i = 2:numel(lines)
        vals = sscanf(lines{i}, '%f').';

        if numel(vals) ~= nMach + 1
            error(['Invalid row length in %s at data line %d. ' ...
                   'Expected %d numbers (1 alpha + %d coeffs), got %d.\n' ...
                   'Line content: %s'], ...
                   filename, i, nMach+1, nMach, numel(vals), lines{i});
        end

        alpha_grid(i-1) = vals(1);
        coeff(i-1, :) = vals(2:end);
    end

    % -------- checks --------
    if any(~isfinite(mach_grid))
        error('Mach grid contains non-finite values in %s', filename);
    end
    if any(~isfinite(alpha_grid))
        error('Alpha grid contains non-finite values in %s', filename);
    end
    if any(~isfinite(coeff(:)))
        error('Coefficient table contains non-finite values in %s', filename);
    end

    if numel(unique(mach_grid)) ~= numel(mach_grid)
        error('Mach grid contains duplicate values in %s', filename);
    end
    if numel(unique(alpha_grid)) ~= numel(alpha_grid)
        error('Alpha grid contains duplicate values in %s', filename);
    end

    % griddedInterpolant prefers ascending sample points
    [mach_grid, mach_order] = sort(mach_grid);
    coeff = coeff(:, mach_order);

    [alpha_grid, alpha_order] = sort(alpha_grid);
    coeff = coeff(alpha_order, :);

    G = griddedInterpolant({alpha_grid, mach_grid}, coeff, 'linear', 'linear');
    Ffun = @(alpha_deg, Mach) G(alpha_deg, Mach);
end
