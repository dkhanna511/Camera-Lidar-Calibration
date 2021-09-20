
from tkinter import *
  
# # loading Python Imaging Library 
from PIL import ImageTk, Image 
  
# # To get the dialog box to open when required  
from tkinter import filedialog 
import os
import glob
import argparse
import shutil

# import defaults
import yaml

with open ('calib_config.yaml') as f:
    config = yaml.full_load(f)



#########################################################################
root = Tk() 
data = {}
images = []




def save_images(*args):
    values = [(index, var.get()) for index, var in data.items()]
#    print(values)
    for index, var in data.items():
        # print(index)
        if var.get() ==1:
            print(index)
    
        #    print(os.path.join(images_dir, data))
            # print(image_name)
            shutil.copy(os.path.join(images_dir, sorted(os.listdir(images_dir))[index]), extracted_images_dir)
            shutil.copy(os.path.join(lidar_points_dir, sorted(os.listdir(lidar_points_dir))[index]), extracted_lidar_points_dir)

    exit(0)


def display_images(images_dir,lidar_points_dir, output_dir,  root):
    
    if not os.path.exists(extracted_images_dir):
        os.mkdir(extracted_images_dir)
 
    if not os.path.exists(extracted_lidar_points_dir):
        os.mkdir(extracted_lidar_points_dir)
 

    filenames = os.listdir(images_dir)
    # print(filenames)
    columns = config["image_select_col"]
    image_count = 0
    window = Toplevel(root)
    width = root.winfo_screenwidth() - 50
    height =root.winfo_screenheight() - 50
    window.wm_geometry(str(width)+"x"+str(height))

    canvas = Canvas(window, width = 1200, height = 600)
    canvas.grid(row=0, column=0, sticky= "news")
    #canvas.place(x=0, y=0)

    vsb = Scrollbar(window, orient="vertical", command=canvas.yview)
    vsb.grid(row=0, column=1, sticky="ns")
    canvas.configure(yscrollcommand= vsb.set, scrollregion = canvas.bbox("all"))

    frame_image = Frame(canvas)
    frame_image.pack(expand=True, fill="both")
    #frame_image.grid_rowconfigure(0, weight = 1)
    #frame_image.grid_columnconfigure(0, weight = 1)
    canvas.create_window((0,0), window=frame_image, anchor="nw")

    
    for index, name in enumerate(filenames):
        # print(os.path.join(images_dir, name))
        image_count += 1
        
        r, c = divmod(image_count - 1, columns)
        # print(r, c)
        im = Image.open(os.path.join(images_dir, name))
        resized = im.resize((150, 150), Image.ANTIALIAS)
        tkimage = ImageTk.PhotoImage(resized)
        myvar = Label(frame_image, image = tkimage)
        myvar.image = tkimage
        # myvar.grid(row=r, column = c)
        var = IntVar()
        print(var)
        # name_without_ext = name.split('.')[0]
        # print(name_without_e)
        Checkbutton(frame_image, image = tkimage, variable = var, width = 80, height =100).grid(row=r, column=c)
        data[index] = var
    # print(data)
    myButton = Button(frame_image, text = "Save", command=save_images, fg = "black", pady = 50).grid()# command= myClick)
    # myButton.pack()


    #print "here"
    window.mainloop()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--images_dir', required=True, type=str, help='images_folder')
    parser.add_argument('--lidar_points_dir', required=True, type=str, help='lidar_points_folder')
    parser.add_argument('--output_dir', required=True, type=str, help='Extracted images folder would be kept inside it')

    args = parser.parse_args()
    images_dir = args.images_dir
    lidar_points_dir = args.lidar_points_dir
    output_dir = args.output_dir
    extracted_images_dir = os.path.join(output_dir, "extracted_images")
    extracted_lidar_points_dir = os.path.join(output_dir, "extracted_lidar_points")
    display_images(images_dir,lidar_points_dir, output_dir, root)

