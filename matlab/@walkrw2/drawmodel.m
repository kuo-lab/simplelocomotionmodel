function h = drawmodel(w,x,footposn,h)
% drawmodel(w, x, footposn) draws the 2-d rimless wheel with the foot located at
%   footposn
% h = drawmodel(w) draws the model at its fixed point and a position zero
%   and returns the handle to the graphic

% Art Kuo

if nargin < 2
  x = get(w, 'xstar'); % assume fixed point if no other info
end

if nargin < 3
  footposn = [0; 0];
end

q1 = x(1);

parms = get(w,'parms'); alpha = parms.alpha;

nlegs = floor(pi / (2*alpha)); % roughly the right number of legs
hnlegs = floor(nlegs / 2); % half the number of legs

% form matrices of the x and y positions of line segments
% where one column is a list of positions for a connected line
% and each additional column is a separate line

hubx = -sin(q1) + footposn(1); huby = cos(q1) + footposn(2);
angles = (-hnlegs:hnlegs) *2*alpha;
legsx = [sin(-q1 + angles); sin(-q1 + angles + pi)] + hubx;
legsy = [cos(-q1 + angles); cos(-q1 + angles + pi)] + huby;


if nargin < 4,
  h = line(legsx, legsy,'LineWidth',1,'EraseMode','background','Color','Blue'); %original
else
  for j = 1:length(h)
    set(h(j),'Xdata',legsx(:,j), 'Ydata', legsy(:,j));
  end
end

