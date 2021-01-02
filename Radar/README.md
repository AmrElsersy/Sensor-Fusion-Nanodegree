# Radar-Target-Generation-Detection
Fourth Project of SFND

## Project Layout

<img src="/images/layout.png" width="700" />

1. Configure the FMCW waveform based on the system requirements.
2. Define the range and velocity of target and simulate its displacement.
3. For the same simulation loop process the transmit and receive signal to determine the beat signal
4. Perform Range FFT on the received signal to determine the Range
5. Towards the end, perform the CFAR processing on the output of 2nd FFT to display the target.


## Output 
<img src="/images/1.png" width="700" />


## CFAR implementation

- Training Cells x & y directions corresponding to range & Doppler dimension.
```
Tr = 12;
Td = 6;
```


-Guard Cells in x & y directions around the Cell under test 
```
Gr = 6;
Gd = 3;
```


- Add offset 
```
offset = 7;
```

- a loop such that it slides over the Cell under test through Range-Doppler-Map 
- iterativly .. it sums the signal level in the training cells. then uses db2pow to convert the value from logarithmic to linear . 
- Sum values from the training + gaurd + CUT
- Sum values from the gaurd + CUT
- calculate noise level by substract both of them
- Average these values for the training cells. then convert it to logarithimic using pow2db.
- Add the offset to calculate the threshold. 
- Compare the signal under CUT with the threshold. If CUT level > threshold make it = 1, else make it = 0.
 
```
for i=1:(Nr/2-(2*Gr+2*Tr))
    for j=1:(Nd-(2*Gd+2*Td))
	% Sum values from the training + gaurd + CUT
        s1 = sum(db2pow(RDM(i:i+2*Tr+2*Gr, j:j+2*Td+2*Gd)),'all');
	% Sum values from the gaurd + CUT
        s2 = sum(db2pow(RDM(i+Tr:i+Tr+2*Gr, j+Td:j+Td+2*Gd)),'all');    
	% calculate noise level by substract both of them
        noise_level = s1 - s2;
        
	% avarage then add offset to calculate threshold
        th = noise_level/num_cells;
        th = pow2db(th) + offset;
        th= db2pow(th);
        
        
        signal = db2pow(RDM(i+Tr+Gr, j+Td+Gd));

        if (signal <= th)
            signal = 0;
        else 
            signal = 1;
        end
        
        signal_cfar(i+Tr+Gr,j+Td+Gd) = signal;        
    end
end
```


- display CFAR output
```
figure,surf(doppler_axis,range_axis,RDM);
colorbar;
```

