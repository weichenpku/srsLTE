
%usrp
%file='chestinfo.csv';  nrb=25;
%file='chestinfo_cfo.csv'; 
%file='3.28.20-39-27-0.csv'; nrb=100;
%file='7.25.8-5-31-1.csv';   nrb=75;
%file='4.4.15-35-25-1.csv'; nrb=25;
%file='4.4.19-52-52-1.csv'; nrb=25;

%iris
%file='4.11.19-25-4-0.csv'; nrb=100;
file='4.11.19-26-31-0.csv'; nrb=100;
file='4.14.19-28-32-0.csv'; nrb=50;
file='4.15.10-48-12-1.csv'; nrb=100;
file='4.15.10-50-12-0.csv'; nrb=100;
%file='4.15.20-3-52-1.csv'; nrb=25;


path='/media/soar/My Passport/data/csi_data/';
fd=fopen([path file]);
cin=textscan(fd,"%f,%f,");
creal=cin{1,1};
cimag=cin{1,2};
h=creal+1i*cimag;

subcarrier = 12*nrb;
symbolnum = size(h,1)/subcarrier;

h_est=reshape(h(1:subcarrier*symbolnum),[subcarrier,symbolnum]);

h_abs=abs(h_est);
p=log(h_abs.^2)/log(10)*10;
h_ang=angle(h_est);
mesh(p); figure; mesh(unwrap(h_ang'));
 
h_shape=size(h_est);
% CIR and doppler spread
CIR=ifft(h_est,[],1); % CIR time, symbol time
x=linspace(-1/1500,1/1500,h_shape(1));
y=(1:h_shape(2));
figure; mesh(y,x,abs(fftshift(CIR,1))); title('CIR');

PDP = mean(CIR.^2,2); % CIR time
figure; plot(x,fftshift(abs(PDP))); title('PDP');

DS = fft(CIR,[],2); % CIR time, spead freq
y=linspace(-14000/2,14000/2,h_shape(2));
figure; mesh(y,x,abs(fftshift(fftshift(DS,1),2))); title('Doppler Spread');

