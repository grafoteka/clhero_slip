%%SCRIPT TO SAVE DATA AND LAUNCH EVERY SIMULATION NEEDED %%
%% CLEAN
clc;

%%INITIAL PARAMETERS
cuenta=1;
while(cuenta<3)
    if cuenta== 1
        leg_size = 0.203; % m
        stiffness = 5000; % N/m
    end
    if cuenta==2
        leg_size = 0.160;% m
        stiffness = 8166;% N/m
    end
    
compresion_inicial = leg_size - 10*9.8/stiffness;
for(i = 1:10:200)
    Kp(i) = i
end
%Kp=100; %MODIFICA EL VALOR DE KP y quita algunos ángulos de barrido si quieres o velocidades porque si no te vas a tirar la vida. Ten en cuenta que te genera un monton de figures. 
%% En plotting.m puedes cambiar si quieres ver las gráficas o quitarlas. Si no quieres sacarlas, puedes decirle que te las guarde como .png descomentando lo que hay debajo. 
%El matfilename en ese caso creo que lo vas a tener que modificar. porque
%te dectecta la extensión del archivo mal. Pero bueno eso solo si decides
%quitarle la visibilidad a los plots dentro de plotting. Eso se hace fig =
%figure('visible','on'); poniendolo en off. 

maxtime = 15;

for vel_objetivo=0.1:0.1:1.5
    for angle_running=90:5:110
        for angle_walking=45:90
            [data,performance]=SLIP_model([0,compresion_inicial,vel_objetivo,0,angle_running,angle_walking,stiffness],vel_objetivo, maxtime,leg_size,Kp); 
        %     matfilename=['figures/',num2str(leg_size,'%.3f'),'/SLIP_',num2str(leg_size,'%.3f'),'_',num2str(stiffness,'%.f'),'_',num2str(angle_walking,'%.f'),'_',num2str(angle_running,'%.f'),'_',num2str(vel_objetivo,'%.2f')];

            %if strcmp(performance.terminationmsg,'Simulation time exceeded')
            %    fig = plotting(data,vel_objetivo, leg_size, angle_walking, angle_running,Kp);
        %         print(fig,matfilename,'-dpng','-r500');
            %end
        end
    end
end
cuenta = cuenta +1;
end


