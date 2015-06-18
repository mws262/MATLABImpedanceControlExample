function KeyPress(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%   Key: name of the key that was pressed, in lower case
%   Character: character interpretation of the key(s) that was pressed
%   Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
global tarControl

tarControl = ~tarControl;

if tarControl
   disp('Mouse will change the target point of the end effector.')
else
   disp('Mouse will apply a force on end effector.') 
end