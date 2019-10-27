# Artificial_Artist
Robotics Arm Drawing Bot
1. combined_sync.py - main code to control the arm with syncwrite
2. combined_write.py - main code to control the arm without syncwrite, but with variable speed
3. draw.py - Image processing for the drawing from user
4. ik.py - Inverse Kinematics for the robot arm
5. play_sound.py - play a sound to notify the drawing has been done
6. skip.py - Reduce the output point from draw.py
7. reference.py - reference code from Senior

## To run the robot arm:
##### 1. Goal from User_input:
    'python combined_write.py'
##### 2. Goal from the program 
    'python combined_write.py --user_input'

## Things to be done:
1. image from judges
2. fix the pen
3. colour paper to decorate the base
4. Test the workspace and minimum wait time between points with power suppy
5. Further reduce the points, push the limit
6. Estimate the time taken for 100 points
7. Presentation & Report


