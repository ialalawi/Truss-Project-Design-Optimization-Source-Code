% Truss Project Computational Analysis Algorithm V3
% Developer: Ismaeel AlAlawi, copyright © 2020 all rights reserved. 

%{ 
Truss Verification Problem Member Index Definition:
AB = 1     BC = 4   DE = 7  EG = 10 GH = 13
AD = 2     CD = 5   CE = 8  FG = 11
BD = 3     DF = 6   EF = 9  FH = 12

Joint index specified by alphabetical order of joints. (i.e. A = 1, 
B = 2, C = 3, etc.)

There are 8 joints and 13 members in the problem. 
%}

clear % clear variables

% Define Connection Matrix C
C = [1 1 0 0 0 0 0 0 0 0 0 0 0; 1 0 1 1 0 0 0 0 0 0 0 0 0;...
    0 0 0 1 1 0 0 1 0 0 0 0 0; 0 1 1 0 1 1 1 0 0 0 0 0 0;...
    0 0 0 0 0 0 1 1 1 1 0 0 0; 0 0 0 0 0 1 0 0 1 0 1 1 0;...
    0 0 0 0 0 0 0 0 0 1 1 0 1; 0 0 0 0 0 0 0 0 0 0 0 1 1];


% Define Reaction Force Matrices Sx and Sy 
Sx = [1 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0];
Sy = [0 1 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 1];


% Define Joint Location Vectors X and Y (point A taken as origin)
X = [0 0 4 4 8 8 12 12];
Y = [0 4 8 4 8 4 4 0];

% Define Applied External Load Vector L
L = [0 0 0 0 0 0 0 0 0 0 0 25 0 0 0 0]'; % Load located at J4 (pin D)


% Save variables in .mat file
save trussverificationproblem C Sx Sy X Y L 

% Define Coefficient Matrix A
endcolA = [Sx; Sy]; % define last three columns of A

[J,M] = find(C); % get joint, member connection matrix indices
[r,c] = size(C); % get connection matrix size
mat_J = []; % pre-allocate coordinate difference matrix

%{
 Calculate xdiff and ydiff and store them in a matrix, where each row
 corresponds to the truss member index.
%}
for i = 1:2:length(J) 
    mat_J = [mat_J; X(J(i+1)) - X(J(i)), Y(J(i+1)) - Y(J(i))]; 
end

mat_Jp = -mat_J; % define second joint mat for a given member

J = reshape(J, [2,c]); % reshape for easier indexing for coefficient mat

% Define Ax (x-component of A)
for j = 1:c
    i = J(:,j);
    A(i(1),j) = mat_J(j,1)/norm(mat_J(j,:)); % get first joint xdiff/norm
    A(i(2),j) = mat_Jp(j,1)/norm(mat_J(j,:)); % get second joint xdiff/norm
end

% Define Ay (y-component of A)
for j = 1:c
    i = J(:,j);
    A(i(1)+r,j) = mat_J(j,2)/norm(mat_J(j,:)); % get first joint ydiff/norm
    A(i(2)+r,j) = mat_Jp(j,2)/norm(mat_J(j,:)); % get second joint ydiff/norm
end

A = [A endcolA]; % define A completely by horizontal concatenation

% Find Force Matrix T
T = A\L;
T(round(T)==0) = 0; % assign zero to both 0 and -0 (different IEEE representations)
disp('\% EK301, Section A2, GIM Engineers: Ismaeel A., Gokai W., Michael A. 04/03/2020.')
% Print Applied External Load and Member forces
fprintf('Load: %.f N\n', L(L ~= 0));
disp('Member forces in Newtons')
for i = 1:c
    if T(i) == 0 % determine which members are under compression/tension
        state = '';
    elseif T(i) > 0
        state = 'T';
    else
        state = 'C';
    end
    fprintf('m%d: %.3f (%c)\n', i, T(i), state); % print member forces
end

% Print Reaction Forces
disp('Reaction forces in Newtons:') 
fprintf('Sx1: %.2f\n', T(end-2))
fprintf('Sy1: %.2f\n', T(end-1))
fprintf('Sy2: %.2f\n', T(end))

% Print Cost
lengths = sqrt(sum(mat_J.^2,2)); % define truss member lengths
cost = 10 * r + sum(lengths); % calculate truss cost
fprintf('Cost of truss: $%.2f\n', cost) % print truss cost
 
% Print Max Load/Cost Ratio
Compress = abs(T(T < 0)); % define member under compression vector
compression_lengths = lengths(T(1:end-3)<0); % define compression lengths
SR = Compress ./ buckling(compression_lengths); % define SR vector
criticalmember = find(ismember(T, -Compress(SR == max(SR)))); % critical member(s)
maxload = L(L~=0)/max(SR); % find maxload
fprintf('Theoretical max load/cost ratio in N/$: %.4f\n', maxload/cost);
