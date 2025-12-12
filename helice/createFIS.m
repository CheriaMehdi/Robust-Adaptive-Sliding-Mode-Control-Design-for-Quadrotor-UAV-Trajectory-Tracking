function fis = createFIS()
    % Use Sugeno FIS (Takagi-Sugeno) — required for constant output MFs
    fis = sugfis;

    % Add inputs
    fis = addInput(fis, [-2 2], 'Name', 'S');
    fis = addInput(fis, [-5 5], 'Name', 'dS');

    % Add output (NO 'NumMembershipFunctions' allowed!)
    fis = addOutput(fis, [0.1 2.0], 'Name', 'k1');

    % Input MFs for S
    fis = addMF(fis, 'S', 'gaussmf', [0.8, -1.6], 'Name', "NL");
    fis = addMF(fis, 'S', 'gaussmf', [0.8, -0.8], 'Name', "NS");
    fis = addMF(fis, 'S', 'gaussmf', [0.8,  0.0], 'Name', "ZE");
    fis = addMF(fis, 'S', 'gaussmf', [0.8,  0.8], 'Name', "PS");
    fis = addMF(fis, 'S', 'gaussmf', [0.8,  1.6], 'Name', "PL");

    % Input MFs for dS
    fis = addMF(fis, 'dS', 'gaussmf', [1.5, -3.0], 'Name', "NL");
    fis = addMF(fis, 'dS', 'gaussmf', [1.5, -1.5], 'Name', "NS");
    fis = addMF(fis, 'dS', 'gaussmf', [1.5,  0.0], 'Name', "ZE");
    fis = addMF(fis, 'dS', 'gaussmf', [1.5,  1.5], 'Name', "PS");
    fis = addMF(fis, 'dS', 'gaussmf', [1.5,  3.0], 'Name', "PL");

    % Output MFs: constant values (allowed in Sugeno only)
    output_levels = [0.15, 0.4, 0.8, 1.4, 1.9]; % VL, L, M, H, VH
    mf_labels = ["VL", "L", "M", "H", "VH"];
    for i = 1:5
        fis = addMF(fis, 'k1', 'constant', output_levels(i), 'Name', mf_labels{i}); % Use {i} for string
    end

    % Rule base from Table 2 in the paper
    % S MF indices: PL=1, PS=2, ZE=3, NS=4, NL=5
    % dS MF indices: NL=1, NS=2, ZE=3, PS=4, PL=5
    % Output levels: VL=1, L=2, M=3, H=4, VH=5

    rule_table = [
        3 2 1 1 1;  % S = PL
        4 3 2 2 1;  % S = PS
        4 4 3 2 2;  % S = ZE
        5 4 4 3 2;  % S = NS
        5 5 5 4 3   % S = NL
    ];

    rules = [];
    for i = 1:5       % loop over S (i = 1 → PL, i = 5 → NL)
        for j = 1:5   % loop over dS (j = 1 → NL, j = 5 → PL)
            out_idx = rule_table(i, j);
            % Sugeno rule format: [input1_mf_index, input2_mf_index, output_mf_index, weight, connection]
             rules(end+1, :) = [i, j, out_idx, 1, 1];
        end
    end

    fis = addRule(fis, rules);
end