function animate4(w, xs, ts, stepindices, h1, h2, varargin)
% animate(w) 

parms = get(w,'parms'); alpha = parms.alpha; LW = 0.5;
xlen = length(xs);


% figPos = [500, 500, 300, 100];%[100, 100, 1800, 1800];
% fig1 = figure('Position', figPos); 
% hold on; 

nom_steplen = 2*sin(alpha);
bumps = get(w,'bumps');

axes(h1);
xlimit = [-2*nom_steplen (length(bumps)+0)*nom_steplen]; xlim(xlimit); % Estimate range of walking
 ylim([-0.1 1.5])
axis equal;


%find and draw bump
bump_len = 1.05*nom_steplen;
bumpsTemp = [bumps(2:end) 0];
bump_magTemp = cos(alpha-bumpsTemp) - cos(alpha+bumpsTemp);
beginBumpX = 0; beginBumpY = 0;
line([xlimit(1) beginBumpX], [0 0],'Color','Black', 'LineWidth',LW);

for i = 1:length(bump_magTemp)
    bumpCoor = [beginBumpX beginBumpY; beginBumpX beginBumpY+bump_magTemp(i); bump_len+beginBumpX beginBumpY+bump_magTemp(i);];
    line(bumpCoor(:,1), bumpCoor(:,2), 'LineWidth', LW,'Color','Blue');
    beginBumpX = beginBumpX +  bump_len;
    beginBumpY = beginBumpY + bump_magTemp(i);
end
line([ beginBumpX  xlimit(2)], [beginBumpY beginBumpY], 'LineWidth', LW,'Color','green');

vidObj = VideoWriter('vid_STAIRS.avi');
% vidObj.FrameRate = 60;
open(vidObj);

h = drawmodel(w, xs(1,:), [0;0]);
bump_mag = cos(alpha-bumps) - cos(alpha+bumps);
footx = 0; footy = 0; stepindicesInd = 1;
for i = 1:10:xlen
    if i > stepindices(stepindicesInd)
        if stepindicesInd ~= length(stepindices)
            stepindicesInd = stepindicesInd + 1;
        end
        footx = footx + sin(abs(xs(i,1))) + sin(  2*alpha-abs(xs(i,1)) ) ; 
        footy =  footy + bump_mag(stepindicesInd); 
    end

    x = xs(i,:);
    drawmodel(w, x, [footx; footy], h);  
    
    hubx = -sin(x(1)) + footx;
    hTemp1 = line([hubx hubx], [ylim], 'LineWidth', LW, 'LineStyle','--', 'Color','black');
    
    axes(h2);
    hTemp2 = line([ts(i) ts(i)], [ylim], 'LineWidth', LW, 'LineStyle','--', 'Color','black');
    
    
%     drawnow; 
    pause(0.05);%pause(0.001);
    delete(hTemp1); delete(hTemp2);
     axes(h1);
     
   M(i) = getframe(gcf);
    writeVideo(vidObj,M(i))
     
%     
end

axes(h1);
hTemp1 = line([hubx hubx], [ylim], 'LineWidth', LW, 'LineStyle','--', 'Color','black');
axes(h2);
hTemp2 = line([ts(i) ts(i)], [ylim], 'LineWidth', LW, 'LineStyle','--', 'Color','black');

% 
% %find and draw bump
% bump_len = 1.1*nom_steplen;
% bump_index = find(bumps ~= 0); 
% bump = bumps(bump_index);
% bump_mag = cos(alpha-bumps) - cos(alpha+bumps);


% if ~isempty(bump_index)
%     
%     coef = 0.95; % this coeff shifts the bump
%     
%     beginBumpX = (bump_index(1)-1)*nom_steplen*coef;
%     beginBumpY = 0;
% 
%     l_left_corner = [beginBumpX; 0];
%     line([xlimTemp(1) l_left_corner(1)], [0 0],'Color','Black', 'LineWidth',LW);
% 
%     for i = 1:length(bump_index)
%         bumpCoor = [beginBumpX beginBumpY; beginBumpX beginBumpY+bump_mag(bump_index(i)); bump_len+beginBumpX beginBumpY+bump_mag(bump_index(i));];
%         hXX = line(bumpCoor(:,1), bumpCoor(:,2), 'LineWidth', LW,'Color','Blue');
%         beginBumpX = beginBumpX +  bump_len;
%         beginBumpY = beginBumpY + bump_mag(bump_index(i));
%     end
%     line([ beginBumpX  xlimit(2)], [beginBumpY beginBumpY], 'LineWidth', LW,'Color','green');
% end