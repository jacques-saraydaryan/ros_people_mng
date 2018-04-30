string color_name
# color name without correction (black, white, gray)
string color_temp
# closest web color name
string color_web
# custom black, white, gray, dark brightness name
string color_brightness_name
# rgb[0]=R, rgb[1]=G, rgb[2]=B
float32[] rgb
# hsv[0]=H, hsv[1]=S, hsv[2]=V
float32[] hsv
# percentage of the current color into the image (after kmean clusterisation)
float32 percentage_of_img