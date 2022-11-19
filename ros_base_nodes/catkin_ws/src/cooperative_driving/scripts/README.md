# Instructions
## Map image drawing
For any different map you should create a folder with at least one image with 1px=1cm resolution in png and bmp formats. It is recommended, if feasible, to create a draw_map script for each.

1. I am used to draw the lane center lines by hand using [inkscape](https://inkscape.org/). For each lane draw the center line and save it as .svg file. 

2. Then, open the file with a text editor, copy the path variable and use it in the [GreenSock SVG challenge](https://codepen.io/GreenSock/pen/zYbddq/ecdfb83c70724638f83376a0cfad6b26).

3. Create a new item in maps_info.py with the previous input. 

4. Modify make_path_new_map.py and process new map item. 

5. Modify make_force_matrixs.py and process previous outcome. 


## TODO
Parse arguments for make_path_new_map.py, make_force_matrixs.py and plotforces.py scripts.