%% size of PCB 
pcbThickness = 1.6e-3;
pcbLength = 152.4e-3;
pcbWidth = 101.6e-3; 

pcbMaterial = 'FR4';
pcbEpsilonR = 4.4;

d = dielectric(pcbMaterial);
d.EpsilonR = pcbEpsilonR;
d.Thickness = pcbThickness; 

AntennaPlane = antenna.Rectangle('Length',0,5e-2, 'Center', [0,0]);
GndPlane = antenna.Rectangle('Length',pcbLength, 'Width',pcbWidth);

p = pcbStack;
p.Name = 'strip-fed slot';
p.BoardingThickness = pcbThickness; 
p.Layers ={AntennaPlane, d,GndPlane}; 
p.FeedbackLocations = [o,(-pcbWidth/2)+6e-3,1,3];

rec = antenna.Rectangle('Length', 10e-2,'Width',2e-2, 'Center', [0,-20e-3]);
rec1 = antenna.Rectangle('Length', 6e-2,'Width',2e-2, 'Center', [0,20e-3]);


AntennaPlane = AntennaPlane + Rec + Rec1; 
p.Layers = {AntennaPlane,d,GndPlane};

figure(1);
show(p);
figure(2);
pattern(p,1.943e9);

figure(3);
impedance(p,1.6e9:2e7:2e9);

freq = linspace(1.6e9, 2.2e9, 50);
s = sparameters(p, freq, 50);

figure(4);
rfplot(s);


