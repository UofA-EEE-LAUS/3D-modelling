%% Map and obstacles
map_dimension = [2000 2000];

obstacles = [[500,50,200,200];
             [300,500,400,600];
             [1800,1600,100,150];
             [100,1000,150,200];
             [1200,400,200,1000];
             [600,1500,600,200];
             [1450,800,350,200]];
%% Trajectory
rrt_trajectory = [
    0,0;
    14.9780508088000,0.811168274160795;
    54.5304524613936,30.1439889756416;
    70.0807248189814,84.0165461228392;
    98.0351058446329,127.095706516207;
    127.011432567202,172.178107950143;
    183.171506623523,192.683320270357;
    211.215963241643,244.254600302940;
    263.695629429634,261.031933023312;
    320.319263681356,257.411123083437;
    356.806403562287,293.143122003868;
    408.416709800996,275.097887683077;
    462.133497101898,296.798044811661;
    521.499478147204,298.447812988874;
    576.194841022051,321.382408321352;
    632.972821509658,324.135215131131;
    690.944152113962,328.486778386522;
    707.699630729678,384.673529935719;
    704.474151707051,437.745690407862;
    723.610403268194,494.080422494094;
    747.203308454544,538.484891707109;
    771.806370727904,591.832236911614;
    795.668236620268,644.664112797641;
    819.251392145602,687.880894156389;
    836.854350786663,742.331251065109;
    836.394580254383,799.526030542235;
    831.197279318189,852.480534849707;
    842.853361711221,904.930951201343;
    849.533744136480,957.757517034632;
    836.427010152600,1005.04764180319;
    806.662971085444,1055.77676666940;
    764.899386215687,1086.55674651733;
    743.986016240931,1137.63091349057;
    724.473317179105,1186.46274337738;
    685.771095634157,1231.33937612642;
    632.880086645921,1253.82271278015;
    595.744437514756,1294.62506292816;
    554.651743338135,1325.99363304987;
    532.281812811924,1372.47838485980;
    509.066209667010,1415.46039653409;
    489.881759643065,1464.57513593227;
    439.267972966984,1482.00173338788;
    390.023729779867,1511.26762707567;
    337.295834107460,1508.86039617845;
    287.191888056247,1511.94294336930;
    237.213377310467,1542.78176485049;
    239.582874845660,1599.97062393341;
    228.726553885762,1653.05542271259;
    195.526724437196,1702.49392743304;
    0,1800];