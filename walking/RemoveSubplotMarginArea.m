function [] = RemoveSubplotMarginArea(subAxes, m,n,p)
% Remove the blank area in the margin of the figure/subplot of the current
% active figure/subplot

% subAxes: handle of the axes of the subplot
% m :   row of the subplot
% n :   column of the subplot
% p :   the index of the current/actived subplot

% Attention: for saving fig, using the following command to set the
% parameter, otherwise, this function cannot change the fig saved
% set(gcf, 'PaperPositionMode', 'auto');

if nargin <4
    p = 1;

end

% ind2sub: Convert linear indices to subscripts. Please note that the
% direction of subplot is different from arrays.
% col : the column of the actived subplot
% row : the row of the actived subplot
[col, row] = ind2sub([n,m],p);

% OuterPosition
% set the left and right margins for the whole figure 
marginLeftRight = 0.01; 
% set the top and bottom margins for the whole figure 
marginTopBottom = 0.1;
% The followin code divide the whole fig to m row and n column
sub_axes_x = (col-1) * (1-2*marginLeftRight)/n                          + marginLeftRight;
sub_axes_y = (1 - 2*marginTopBottom) -row * (1 - 2*marginTopBottom)/m   + marginTopBottom; 
sub_axes_w = (1-2*marginLeftRight)/n;
sub_axes_h = (1-2*marginTopBottom)/m;

% set the margin for title, xlabel, ylabel
delt_w = 0.05 * 1/n;
delt_h = 0.05 * 1/m;
% pos = [sub_axes_x + delt_w, sub_axes_y + delt_h,...
%                 sub_axes_w - delt_w, sub_axes_h - 2* delt_h]
set(subAxes, 'OuterPosition', [sub_axes_x + delt_w, sub_axes_y + delt_h,...
                sub_axes_w - delt_w, sub_axes_h - 2* delt_h]); % OuterPosition


% TightInset
inset_vectior = get(subAxes, 'TightInset');
% inset_vectior = get(gca, 'Position');
inset_x = inset_vectior(1);
inset_y = inset_vectior(2);
inset_w = inset_vectior(3);
inset_h = inset_vectior(4);

% OuterPosition
outer_vector = get(subAxes, 'OuterPosition');
pos_new_x = outer_vector(1) + inset_x; % move the orignal point of Position to that of the TightInset
pos_new_y = outer_vector(2) + inset_y;
pos_new_w = outer_vector(3) - inset_w - inset_x; % reset the width of Position
pos_new_h = outer_vector(4) - inset_h - inset_y; % reset the height of Position

% Position
set(subAxes, 'Position', [pos_new_x, pos_new_y, pos_new_w, pos_new_h]);


end


