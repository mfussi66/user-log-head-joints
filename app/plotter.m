C = csv2cell('wrist.csv');
S = cell2struct( C(2:end,:).', C(1,:) );

t = [S.Time];

yaw = [S.Yaw];
pitch = [S.Pitch];
roll = [S.Roll];

m0o= [S.Mot0_Out];
m1o= [S.Mot1_Out];
m2o= [S.Mot2_Out];

m0o= [S.Mot0_Out];
m1o= [S.Mot1_Out];
m2o= [S.Mot2_Out];
