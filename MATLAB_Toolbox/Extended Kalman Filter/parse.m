function M = parse(D, expectedCols)
%PARSE Normalize logged Simulink signal to N×C (rows=time).
%   M = PARSE(D)              -> squeezes/log reshapes to N×C heuristically.
%   M = PARSE(D, expectedCols)-> also validates size(M,2)==expectedCols.
%
% Handles common Simulink logging shapes:
% - [N×C], [C×N]
% - [1×C×N], [C×1×N], [N×C×1], [C×N×1]
% - [1×1×N] (scalar per sample) -> N×1
% - Vectors [N×1], [1×N]
%
% Strategy:
% 1) If ndims(D)>3, error.
% 2) If 3D, try to identify time dimension as the largest dim and move it to rows.
% 3) Squeeze to 2D, then orient to N×C (prefer columns=3,4,6.. if expected or common).
% 4) If expectedCols given and we have a single column, attempt auto-fix if total elements fits.

    if nargin < 2
        expectedCols = [];
    end
    if isempty(D)
        error('parse:EmptyInput', 'Input data is empty.');
    end

    % Accept at most 3D (Simulink log)
    nd = ndims(D);
    if nd > 3
        error('parse:BadDims', 'Input has %d dims; expected ≤ 3.', nd);
    end

    A = D;

    % If 3D, rearrange to put time in rows
    if nd == 3
        sz = size(A);
        % Identify time dimension: typically the largest dimension for logged data
        [~, tdim] = max(sz);
        switch tdim
            case 1
                % time already at rows: [N × ? × ?]
                % collapse remaining dims into columns
                A = reshape(A, sz(1), []);
            case 2
                % time at columns: permute to rows
                A = permute(A, [2 1 3]);  % move time to dim-1
                szp = size(A);
                A = reshape(A, szp(1), []); % collapse other dims
            case 3
                % time at pages: permute to rows
                A = permute(A, [3 1 2]);  % move time to dim-1
                szp = size(A);
                A = reshape(A, szp(1), []); % collapse other dims
        end
    end

    % Now squeeze (should be 2D)
    A = squeeze(A);
    if ~ismatrix(A)
        error('parse:Not2D', 'Could not reduce to 2D; size=%s.', mat2str(size(A)));
    end

    % If vector, make it N×1
    if isvector(A)
        A = A(:); % column vector N×1
    end

    % Orient to N×C (rows=time)
    [r, c] = size(A);

    % Heuristic orientation if no expectedCols
    if isempty(expectedCols)
        commonC = [3,4,6,9];
        if ismember(c, commonC)
            % already N×C
        elseif ismember(r, commonC)
            A = A.'; [r,c] = size(A);
        else
            % time is typically longer than components
            if r < c
                A = A.'; [r,c] = size(A);
            end
        end
    else
        % Try to make columns match expectedCols
        if c == expectedCols
            % OK
        elseif r == expectedCols
            A = A.'; [r,c] = size(A);
        elseif c ~= expectedCols
            % Auto-fix attempt for degenerate shapes:
            % If we have N×1 but total elements divisible by expectedCols,
            % try to reshape to N×expectedCols
            total = numel(A);
            if c == 1 && mod(total, expectedCols) == 0
                newN = total / expectedCols;
                A = reshape(A, newN, expectedCols);
                [r,c] = size(A);
            elseif r == 1 && mod(total, expectedCols) == 0
                A = A.'; % 1×N -> N×1 first
                total = numel(A);
                newN = total / expectedCols;
                A = reshape(A, newN, expectedCols);
                [r,c] = size(A);
            else
                % Last chance: if 3×N came in as N×1 by column stacking,
                % the above reshape handles it; otherwise bail out.
            end
        end

        % Final validation
        if size(A,2) ~= expectedCols
            error('parse:WrongCols', 'Expected %d columns, got %d. Size=%s', ...
                  expectedCols, size(A,2), mat2str(size(A)));
        end
    end

    % Sanity checks
    if any(~isfinite(A(:)))
        warning('parse:NonFinite', 'Output contains non-finite values (NaN/Inf).');
    end

    M = A;
end
