function h = drawmodel_double(w1, x1, w2, x2, footposn, h)
% drawmodel(w, x, footposn) draws the 2-d rimless wheel with the foot located at
%   footposn
% h = drawmodel(w) draws the model at its fixed point and a position zero
%   and returns the handle to the graphic

% Art Kuo

% if nargin < 2
%   x = get(w, 'xstar'); % assume fixed point if no other info
% end
% 
% if nargin < 3
%   footposn = [0; 0];
% end



parms = get(w1,'parms'); alpha = parms.alpha;

nlegs = floor(pi / (2*alpha)); % roughly the right number of legs
hnlegs = floor(nlegs / 2); % half the number of legs

% form matrices of the x and y positions of line segments
% where one column is a list of positions for a connected line
% and each additional column is a separate line

q1_1 = x1(1);
hubx = -sin(q1_1) + footposn(1); huby = cos(q1_1) + footposn(2);
angles_1 = (-hnlegs:hnlegs) *2*alpha;
legsx_1 = [sin(-q1_1 + angles_1); sin(-q1_1 + angles_1 + pi)] + hubx;
legsy_1 = [cos(-q1_1 + angles_1); cos(-q1_1 + angles_1 + pi)] + huby;

q1_2 = x2(1);
hubx = -sin(q1_2) + footposn(1); huby = cos(q1_2) + footposn(2);
angles_2 = (-hnlegs:hnlegs) *2*alpha;
legsx_2 = [sin(-q1_2 + angles_2); sin(-q1_2 + angles_2 + pi)] + hubx;
legsy_2 = [cos(-q1_2 + angles_2); cos(-q1_2 + angles_2 + pi)] + huby;


legsx = [legsx_1 legsx_2];
legsy = [legsy_1 legsy_2];

if nargin < 6,
  h = line(legsx, legsy, 'LineWidth',2,'EraseMode','background','Color','Blue');
else
  for j = 1:length(h)
    set(h(j),'Xdata',legsx(:,j), 'Ydata', legsy(:,j));
  end
end

