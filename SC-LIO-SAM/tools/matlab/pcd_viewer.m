pcd_dir = "/home/user/Desktop/data/kaist01/sc-lio-sam/Scans/";

pcd_names = listdir(pcd_dir);

% sort names 
pcd_names_idx = [];
for ii = 1:length(pcd_names)
    pcd_idx = str2double(pcd_names{ii}(1:end-4));
    pcd_names_idx = [pcd_names_idx, pcd_idx];
end
[~, sorted_loc] = sort(pcd_names_idx);

pcd_names_sorted = pcd_names(sorted_loc);

% viz 
for ii = 1:length(pcd_names)

    figure(1);clf;
    pc = pcread(fullfile(pcd_dir, pcd_names_sorted{ii}));
    pcshow(pc);

    title(pcd_names_sorted{ii});

    caxis([0, 200]);
    
    xlim([-80, 80]);
    ylim([-80, 80]);
    zlim([-2, 15]);

end