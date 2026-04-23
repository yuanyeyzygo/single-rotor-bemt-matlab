clc;
clear;

% preprocess_rotor_lookup_data
%
% Offline preprocessing script.
%
% Run this FIRST in normal MATLAB.
% It reads:
%   - CS1_cl.txt ... CS6_cl.txt
%   - CS1_cd.txt ... CS6_cd.txt
%   - chord_interp.mat   (variable F)
%   - pretwist_interp.mat (variable pre_twist)
%
% and writes:
%   rotor_lookup_preprocessed.mat
%
% The output MAT contains only numeric arrays, ready to be inserted into
% rotor for the codegen-clean runtime solver.

%% User settings
max_id = 6;
x_lookup_sample = linspace(0, 1, 201).';   % for chord / pretwist sampling

%% Airfoil C81 txt files
cl_files = { ...
    'CS1_cl.txt','CS2_cl.txt','CS3_cl.txt', ...
    'CS4_cl.txt','CS5_cl.txt','CS6_cl.txt'};

cd_files = { ...
    'CS1_cd.txt','CS2_cd.txt','CS3_cd.txt', ...
    'CS4_cd.txt','CS5_cd.txt','CS6_cd.txt'};

[airfoil_alpha_grid, airfoil_mach_grid, airfoil_cl_table, airfoil_cd_table] = ...
    preprocess_airfoil_tables(cl_files, cd_files, max_id);

%% Chord lookup mat
chord_mat = 'chord_interp.mat';
chord_var = 'F';
%chord_unit_code = int32(2); % 1 meter, 2 c_over_R 

[chord_x_grid, chord_lookup_val] = preprocess_1arg_lookup_from_mat( ...
    chord_mat, chord_var, x_lookup_sample, 'chord');

%% Pretwist lookup mat
pretwist_mat = 'pretwist_interp.mat';
pretwist_var = 'pre_twist';

[pretwist_x_grid, pretwist_lookup_val] = preprocess_1arg_lookup_from_mat( ...
    pretwist_mat, pretwist_var, x_lookup_sample, 'pretwist');

%% Save numeric-only output

fprintf('\nSaved: rotor_lookup_preprocessed.mat\n');
fprintf('  airfoil_alpha_grid : [%d x 1]\n', numel(airfoil_alpha_grid));
fprintf('  airfoil_mach_grid  : [1 x %d]\n', numel(airfoil_mach_grid));
fprintf('  airfoil_cl_table   : [%d x %d x %d]\n', size(airfoil_cl_table,1), size(airfoil_cl_table,2), size(airfoil_cl_table,3));
fprintf('  airfoil_cd_table   : [%d x %d x %d]\n', size(airfoil_cd_table,1), size(airfoil_cd_table,2), size(airfoil_cd_table,3));
fprintf('  chord_x_grid       : [%d x 1]\n', numel(chord_x_grid));
fprintf('  pretwist_x_grid    : [%d x 1]\n', numel(pretwist_x_grid));
rotor = struct();
rotor.R = 1.4;
rotor.Nb = 4;
rotor.omega = 90;
rotor.rho = 1.225;
rotor.I_beta = 2;
rotor.k_beta = 15000;
rotor.tilt_angle_deg = 90.0;
rotor.rotational_direction = 1;
rotor.hub_loc = [0; 0; 0];

rotor.root_cutout = 0.1;
rotor.n_be = 10;
rotor.n_az = 72;

%% Spanwise sectioning
rotor.section_r_end = [1 1 1 1 1 1];
rotor.section_airfoil_id = [1 1 1 1 1 1];

%% MODEL SELECTION
% 1 = linear
% 2 = lookup
rotor.airfoil_mode_code = 1;
rotor.chord_mode_code = 1;
rotor.chord_unit_code = int32(1); % 1 meter, 2 non-dimentional 
rotor.pretwist_mode_code = 1;

%% AIRFOIL
rotor.airfoil_a0 = 5.7 * ones(1, max_id);
rotor.airfoil_alpha0_deg = zeros(1, max_id);
rotor.airfoil_cd0 = 0.01 * ones(1, max_id);
rotor.airfoil_k = 0.02 * ones(1, max_id);

rotor.airfoil_alpha_grid = airfoil_alpha_grid;
rotor.airfoil_mach_grid = airfoil_mach_grid;
rotor.airfoil_cl_table = airfoil_cl_table;
rotor.airfoil_cd_table = airfoil_cd_table;

%% CHORD
rotor.chord_root_value = 0.18;
rotor.chord_tip_value = 0.18;
rotor.chord_x_grid = chord_x_grid;
rotor.chord_lookup_val = chord_lookup_val;

%% PRETWIST
rotor.pretwist_root_deg = 0.0;
rotor.pretwist_tip_deg = -12.0;
rotor.pretwist_x_grid = pretwist_x_grid;
rotor.pretwist_lookup_val = pretwist_lookup_val;

save('rotor.mat','rotor');

function [alpha_ref, mach_ref, cl_table, cd_table] = preprocess_airfoil_tables(cl_files, cd_files, max_id)
    if numel(cl_files) < max_id || numel(cd_files) < max_id
        error('Need at least %d CL/CD file pairs.', max_id);
    end

    alpha_ref = [];
    mach_ref = [];
    cl_table = [];
    cd_table = [];

    for k = 1:max_id
        if exist(cl_files{k}, 'file') ~= 2
            error('Missing file: %s', cl_files{k});
        end
        if exist(cd_files{k}, 'file') ~= 2
            error('Missing file: %s', cd_files{k});
        end

        [alpha_cl, mach_cl, cl_k] = read_c81_table_txt_preprocess(cl_files{k});
        [alpha_cd, mach_cd, cd_k] = read_c81_table_txt_preprocess(cd_files{k});

        if k == 1
            alpha_ref = alpha_cl;
            mach_ref = mach_cl;
            cl_table = zeros(numel(alpha_ref), numel(mach_ref), max_id);
            cd_table = zeros(numel(alpha_ref), numel(mach_ref), max_id);

            cl_table(:,:,k) = cl_k;

            if same_grid_preprocess(alpha_cd, mach_cd, alpha_ref, mach_ref)
                cd_table(:,:,k) = cd_k;
            else
                cd_table(:,:,k) = remap_table_to_reference_grid_preprocess( ...
                    alpha_cd, mach_cd, cd_k, alpha_ref, mach_ref);
            end
        else
            if same_grid_preprocess(alpha_cl, mach_cl, alpha_ref, mach_ref)
                cl_table(:,:,k) = cl_k;
            else
                cl_table(:,:,k) = remap_table_to_reference_grid_preprocess( ...
                    alpha_cl, mach_cl, cl_k, alpha_ref, mach_ref);
            end

            if same_grid_preprocess(alpha_cd, mach_cd, alpha_ref, mach_ref)
                cd_table(:,:,k) = cd_k;
            else
                cd_table(:,:,k) = remap_table_to_reference_grid_preprocess( ...
                    alpha_cd, mach_cd, cd_k, alpha_ref, mach_ref);
            end
        end
    end
end



function [x_grid, y_grid] = preprocess_1arg_lookup_from_mat(matfile, varname, x_sample, label)
    if exist(matfile, 'file') ~= 2
        error('Missing mat file: %s', matfile);
    end

    S = load(matfile);
    if ~isfield(S, varname)
        error('Variable "%s" is missing in %s.', varname, matfile);
    end

    raw = S.(varname);

    if isa(raw, 'function_handle')
        x_grid = x_sample(:);
        y_grid = zeros(size(x_grid));
        for i = 1:numel(x_grid)
            y_grid(i) = raw(x_grid(i));
        end
        return;
    end

    if isa(raw, 'griddedInterpolant')
        x_grid = x_sample(:);
        y_grid = zeros(size(x_grid));
        for i = 1:numel(x_grid)
            y_grid(i) = raw(x_grid(i));
        end
        return;
    end

    if isstruct(raw)
        if isfield(raw, 'x') && isfield(raw, 'y')
            x_grid = raw.x(:);
            y_grid = raw.y(:);
            [x_grid, y_grid] = validate_xy_preprocess(x_grid, y_grid, label);
            return;
        else
            error('%s lookup struct must contain fields x and y.', label);
        end
    end

    if isnumeric(raw)
        if ismatrix(raw) && size(raw,2) == 2
            x_grid = raw(:,1);
            y_grid = raw(:,2);
            [x_grid, y_grid] = validate_xy_preprocess(x_grid, y_grid, label);
            return;
        elseif isvector(raw)
            y_grid = raw(:);
            x_grid = linspace(0, 1, numel(y_grid)).';
            [x_grid, y_grid] = validate_xy_preprocess(x_grid, y_grid, label);
            return;
        else
            error('%s lookup numeric data must be Nx2 or a vector.', label);
        end
    end

    error('Unsupported lookup class for %s: %s', label, class(raw));
end


function [x_grid, y_grid] = validate_xy_preprocess(x_grid, y_grid, label)
    x_grid = x_grid(:);
    y_grid = y_grid(:);

    if numel(x_grid) ~= numel(y_grid)
        error('%s lookup x/y lengths do not match.', label);
    end
    if any(~isfinite(x_grid)) || any(~isfinite(y_grid))
        error('%s lookup contains non-finite values.', label);
    end
    if numel(unique(x_grid)) ~= numel(x_grid)
        error('%s lookup x grid contains duplicate values.', label);
    end

    [x_grid, idx] = sort(x_grid);
    y_grid = y_grid(idx);
end


function [alpha_grid, mach_grid, coeff] = read_c81_table_txt_preprocess(filename)
    fid = fopen(filename, 'r');
    if fid < 0
        error('Cannot open file: %s', filename);
    end

    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

    lines = {};
    while ~feof(fid)
        tline = fgetl(fid);
        if ischar(tline)
            tline = strtrim(tline);
            if ~isempty(tline)
                lines{end+1} = tline; %#ok<AGROW>
            end
        end
    end

    if numel(lines) < 2
        error('Too few lines in %s.', filename);
    end

    mach_grid = sscanf(lines{1}, '%f').';
    if isempty(mach_grid)
        error('Failed to read Mach grid from %s.', filename);
    end

    nMach = numel(mach_grid);
    nAlpha = numel(lines) - 1;

    alpha_grid = zeros(nAlpha, 1);
    coeff = zeros(nAlpha, nMach);

    for i = 2:numel(lines)
        vals = sscanf(lines{i}, '%f').';
        if numel(vals) ~= nMach + 1
            error('Invalid row length in %s at data line %d.', filename, i);
        end

        alpha_grid(i-1) = vals(1);
        coeff(i-1, :) = vals(2:end);
    end

    if numel(unique(alpha_grid)) ~= numel(alpha_grid)
        error('Duplicate alpha grid values in %s.', filename);
    end
    if numel(unique(mach_grid)) ~= numel(mach_grid)
        error('Duplicate Mach grid values in %s.', filename);
    end

    [mach_grid, im] = sort(mach_grid);
    coeff = coeff(:, im);

    [alpha_grid, ia] = sort(alpha_grid);
    coeff = coeff(ia, :);
end


function tf = same_grid_preprocess(alpha_a, mach_a, alpha_b, mach_b)
    tol = 1e-12;

    tf = isequal(size(alpha_a), size(alpha_b)) && ...
         isequal(size(mach_a), size(mach_b)) && ...
         all(abs(alpha_a(:) - alpha_b(:)) < tol) && ...
         all(abs(mach_a(:) - mach_b(:)) < tol);
end


function coeff_ref = remap_table_to_reference_grid_preprocess(alpha_src, mach_src, coeff_src, alpha_ref, mach_ref)
    F = griddedInterpolant({alpha_src, mach_src}, coeff_src, 'linear', 'linear');

    coeff_ref = zeros(numel(alpha_ref), numel(mach_ref));
    for i = 1:numel(alpha_ref)
        for j = 1:numel(mach_ref)
            coeff_ref(i,j) = F(alpha_ref(i), mach_ref(j));
        end
    end
end
