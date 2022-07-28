dpt = readtable('def_pitch_output.log');
drt = readtable('def_roll_output.log');
rpt = readtable('ret_pitch_output.log');
rrt = readtable('ret_roll_output.log');

variables = {'dt', 'pid', 'pwm', 'enc', 'ref', 'err'};

dpt.Properties.VariableNames = variables;
drt.Properties.VariableNames = variables;
rpt.Properties.VariableNames = variables;
rrt.Properties.VariableNames = variables;

fp = figure();
tiledlayout(2, 2)
ap(1) = nexttile;
stairs(dpt.dt, dpt.enc, 'LineWidth', 1.0)
hold on
stairs(dpt.dt, dpt.ref, 'LineWidth', 1.0)
grid minor
ylabel('Pitch trajectory (deg)')
xlabel('Time (sec)')
ap(2) = nexttile;
stairs(rpt.dt, rpt.enc, 'LineWidth', 1.0)
hold on
stairs(rpt.dt, rpt.ref, 'LineWidth', 1.0)
grid minor
ylabel('Pitch trajectory (deg)')
xlabel('Time (sec)')

ap(1) = nexttile;
stairs(dpt.dt, dpt.err, 'LineWidth', 1.0)
grid minor
ylabel('Pitch error (deg)')
xlabel('Time (sec)')
ap(2) = nexttile;
stairs(rpt.dt, rpt.err, 'LineWidth', 1.0)
grid minor
ylabel('Pitch error (deg)')
xlabel('Time (sec)')


% ----------------------------------------------
fr = figure();
tiledlayout(2, 2);

ap(1) = nexttile;
stairs(drt.dt, drt.enc, 'LineWidth', 1.0)
hold on
stairs(drt.dt, drt.ref, 'LineWidth', 1.0)
grid minor
ylabel('Roll trajectory (deg)')
xlabel('Time (sec)')

ap(2) = nexttile;
stairs(rrt.dt, rrt.enc, 'LineWidth', 1.0)
hold on
stairs(rrt.dt, rrt.ref, 'LineWidth', 1.0)
grid minor
ylabel('Roll trajectory (deg)')
xlabel('Time (sec)')

ar(1) = nexttile;
stairs(drt.dt, drt.err, 'LineWidth', 1.0)
grid minor
ylabel('Roll error (deg)')
xlabel('Time (sec)')
ar(2) = nexttile;
stairs(rrt.dt, rrt.err, 'LineWidth', 1.0)
grid minor
ylabel('Roll error (deg)')
xlabel('Time (sec)')
linkaxes(ap, 'y')
linkaxes(ar, 'y')