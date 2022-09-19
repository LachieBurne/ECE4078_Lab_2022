Google drive link:


Step 1: The file directories should look as follows:

base_dir/
    ./lab_output/
        ./images.txt
	and the images to be tested

    ./calibration/
        ./param/
            ./intrinsic_sim.t
            ./intrinsic.txt

    ./final_weights/
        ./best_sim.pt
        ./best_real.pt
    
    ./TargetPoseEst.py
    ./CV_eval.py
    M3_marking_map.txt
    
Ensure that the images.txt file points to the correct location of the images. In our case we added the images to the lab_output folder as well. 

Step 2: Install the latest version of PyTorch and Torchvision (pip3 install torch)

Step 3: Install sklearn. (pip3 install sklearn.)

Also needs: numpy, pandas, opencv-python (sudo apt-get install python3-opencv)

It may require some other common modules not mentioned here.

Note Well: It is important that these modules are installed to run
with python3. You can use: pip3 install torch

Step 4: The USING_SIM variable needs to either be set to True or False. If using the simulator, set to True, otherwise set to False for the physical robot. 
        This will determine which camera parameters are imported, which model weights are imported, image dimensions for calculations, etc.

Step 5:

Once set up:

python3 TargetPoseEst.py
python3 CV_eval.py

TrueMap.txt needs to be called M3_marking_map.txt


If you have any problems feel free to message us on the slack.
