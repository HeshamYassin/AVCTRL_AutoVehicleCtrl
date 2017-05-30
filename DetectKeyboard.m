%% About this Project
% Autonomous Vehicle AutoPilot System for Navigation in an Unknown Environment
% This project is copyright (c) for ECKE | THINK INNOVATION
% ECKE is an Egyptian Tech Company to provide Solutions to Accelerate
% Industrial Processes using Robotics and IoT
% ECKE AutoPilot System is RESTRICTED Intellictual Property, and no one is
% allowed to use, edit or produce this version for commercial use
% For more information and to contact us, please visit
% http://www.ecke-eg.com
% +20 (0) 102-011-5013

function [ nameC ] = DetectKeyboard( hObject, eventdata, handles )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% display(eventdata);
dataType=eventdata.Character;
AscCode = unicode2native(dataType);
 if ~isempty(AscCode)
    if (AscCode>=33) && (AscCode<=126)
        nameC=eventdata.Character;
%         nameC2=eventdata.Key
%         set(handles.text2,'string',nameC2)
    else%if strcmp(re,'')
        nameC=eventdata.Key;
    end
else
    nameC = eventdata.Key;
end

% ModiNum=length(eventdata.Modifier);
% switch eventdata.Key 
%     %% letter
%     case 'a' 
%         if ModiNum==3
%             set(handles.text1,'String','alt+shift+ctrl+a');
%         elseif ModiNum==2
%             if strcmp(eventdata.Modifier , {'shift' 'alt'})
%                 set(handles.text1,'String','alt+shift+a');
%             elseif strcmp(eventdata.Modifier , {'shift' 'control'})
%                 set(handles.text1,'String','ctrl+shift+a');
%             elseif strcmp(eventdata.Modifier , {'control' 'alt'})
%                 set(handles.text1,'String','ctrl+alt+a');
%             end
%         elseif ModiNum==1
%             if strcmp(eventdata.Modifier , 'alt')
%                 set(handles.text1,'String','alt+a');
% 
%             elseif strcmp(eventdata.Modifier , 'control')
%                 set(handles.text1,'String','ctrl+a');
%             elseif strcmp(eventdata.Modifier , 'shift')
%                 set(handles.text1,'String','shift+a');
%             end
%         else 
%             set(handles.text1,'String','a');
%         end
%         set(handles.text1,'String',nameC);
%         set(handles.AP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.AP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'b' 
%         set(handles.text1,'String',nameC); 
%         set(handles.BP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.BP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'c' 
%         set(handles.text1,'String',nameC); 
%         set(handles.CP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.CP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'd' 
%         set(handles.text1,'String',nameC); 
%         set(handles.DP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.DP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'e' 
%         set(handles.text1,'String',nameC); 
%         set(handles.EP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.EP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'f' 
%         set(handles.text1,'String',nameC);
%         set(handles.FP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.FP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'g' 
%         set(handles.text1,'String',nameC); 
%         set(handles.GP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.GP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'h' 
%         set(handles.text1,'String',nameC); 
%         set(handles.HP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.HP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'i' 
%         set(handles.text1,'String',nameC); 
%         set(handles.IP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.IP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'j' 
%         set(handles.text1,'String',nameC); 
%         set(handles.JP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.JP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'k' 
%         set(handles.text1,'String',nameC);
%         set(handles.KP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.KP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'l' 
%         set(handles.text1,'String',nameC); 
%         set(handles.LP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.LP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'm' 
%         set(handles.text1,'String',nameC); 
%         set(handles.MP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.MP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'n' 
%         set(handles.text1,'String',nameC);
%         set(handles.NP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.NP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'o'
%         set(handles.text1,'String',nameC);
%         set(handles.OP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.OP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'p'
%         set(handles.text1,'String',nameC);
%         set(handles.PP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.PP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'q'
%         set(handles.text1,'String',nameC);
%         set(handles.QP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.QP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'r'
%         set(handles.text1,'String',nameC);
%         set(handles.RP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.RP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 's'
%         set(handles.text1,'String',nameC);
%         set(handles.SP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.SP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 't'
%         set(handles.text1,'String',nameC);
%         set(handles.TP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.TP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'u'
%         set(handles.text1,'String',nameC);
%         set(handles.UP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.UP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'v'
%         set(handles.text1,'String',nameC);
%         set(handles.VP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.VP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'w'
%         set(handles.text1,'String',nameC);
%         set(handles.WP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.WP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'x'
%         set(handles.text1,'String',nameC);
%         set(handles.XP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.XP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'y'
%         set(handles.text1,'String',nameC);
%         set(handles.YP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.YP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%     case 'z'
%         set(handles.text1,'String',nameC);
%         set(handles.ZP,'visible','off','BackgroundColor','k')
%         pause(0.05);
%         set(handles.ZP,'visible','on','BackgroundColor',[0.94,0.94,0.94])
%         
% %% number
%     case '1'
%         set(handles.text1,'String',nameC);
%     case '2'
%         set(handles.text1,'String',nameC);
%     case '3'
%         set(handles.text1,'String',nameC);
%     case '4'
%         set(handles.text1,'String',nameC);
%     case '5'
%         set(handles.text1,'String',nameC);
%     case '6'
%         set(handles.text1,'String',nameC);
%     case '7'
%         set(handles.text1,'String',nameC);
%     case '8'
%         set(handles.text1,'String',nameC);
%     case '9'
%         set(handles.text1,'String',nameC);
%     case '0'
%         set(handles.text1,'String',nameC);
%      
%  %% three modifier
%     case 'shift'
%         set(handles.text1,'String',nameC);
%     case 'control'
%         set(handles.text1,'String',nameC);
%     case 'alt'
%         set(handles.text1,'String',nameC);
%         
%  %% arrow
%     case 'leftarrow'
%         set(handles.text1,'String',nameC);
%     case 'rightarrow'
%         set(handles.text1,'String',nameC);
%     case 'uparrow'
%         set(handles.text1,'String',nameC);
%     case 'downarrow'
%         set(handles.text1,'String',nameC);
%         
%  %% special character in main       
%     case 'tab'
%         set(handles.text1,'String',nameC);
%     case 'capslock'
%         set(handles.text1,'String',nameC);
%     case 'backspace'
%         set(handles.text1,'String',nameC); 
%     case 'windows'
%         set(handles.text1,'String',nameC); 
%     case 'backquote'
%         set(handles.text1,'String',nameC); 
%     case 'hyphen'
%         set(handles.text1,'String',nameC);
%     case 'equal'
%         set(handles.text1,'String',nameC);
%     case 'leftbracket' 
%         set(handles.text1,'String',nameC); 
%     case 'rightbracket'
%         set(handles.text1,'String',nameC); 
%     case 'backslash'
%         set(handles.text1,'String',nameC); 
%     case 'hyphen'
%         set(handles.text1,'String',nameC);
%     case 'semicolon'
%         set(handles.text1,'String',nameC);
%     case 'quote' 
%         set(handles.text1,'String',nameC); 
%     case 'return'
%         set(handles.text1,'String',nameC); 
%     case 'comma'
%         set(handles.text1,'String',nameC);
%     case 'period'
%         set(handles.text1,'String',nameC);
%     case 'slash' 
%         set(handles.text1,'String',nameC); 
% end

end

