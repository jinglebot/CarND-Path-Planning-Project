# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
[//]: # (Media References)
[image1]: ./media/img01.png "Best distance 27.71 mi"
[image2]: ./media/img02.png "Best distance 33.01 mi"
[video1]: ./media/vid1.mp4

### Simulators Used
At first, I tried using the Udacity Workspace for this project. I had to drop it and switch to Docker on my local PC due to several reasons:
1) I was eating up on my GPU hours going nowhere since it took a while for me to piece together a game plan to tackle this project.
2) I tried using Workspace without GPU when writing snippets and save using Workspace with GPU for testing the code. But switching from using Workspace with and without GPU is time-consuming I have to reopen all the files I need everytime I switch.
3) The screen in Workspace is too limiting.
4) I was using Docker on almost all my projects in Term 2 and find it easier to handle.

### Project Progression
I followed the Project Q&A to start and was able to get it to drive straight on the road. Then, I tried implementing additional codes for slowing down, keeping within the lane, changing left and right by adding the **classifier class** and the **vehicle class** from the 'classroom'. Unfortunately, things didn't work out as it should. I had the presumption that we were supposed to use everything that has been discussed in the lessons but from the way the Q&A started off the project, it was more like a DIY, include only what you need from the lessons. I decided to drop the **FSM** but kept the 'cost function'. I also dropped the classifier class for predictions when my car was already getting the needed requirements as stated in the rubric.

### To Do
For me, this project still needs further improvement which I plan to follow up after submission when there are no more time constraints from due dates. There are a lot of areas to cover and a lot of optimize. It is not yet a done deal.

<object width="425" height="350">
![Sample Image 1][image1]
</object>
---


### Video Output

[![Video link][image2]][video1]
