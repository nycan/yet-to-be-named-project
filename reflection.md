# Reflection
## The most difficult part
I would like to say that the most difficult part for me was the problem-solving process or the time spent working on it, but I can't. Through this project, I was entering an ecosystem of packages that I hadn't used before, and those dependency-related problems ended up being the most difficult part to resolve.

For example, due to the codespace not working, I had to temporarily use a virtual machine. My first virtual machine, being arm-based, was not suitable for Webots, which only supports x86 on Ubuntu. I tried to use QEMU emulation only to find out that Webots has a dependency of amd64-based C. A couple hours of playing with the apt-get sources list to fetch amd packages yielded nothing, so I reinstalled Ubuntu and all the necessary packages on a new, x86-based VM. However, all this was only to learn that the emulation was too slow, and the resource-heavy simulation couldn't run at a reasonable framerate on it.

The other issue causing me to tear my hair out was the horrible documentation for Webots. I had originally used Webots because Gazebo didn't work and it seemed to be a fairly prominent simulation. This meant that I spent ages poring over my `my_world.wbt` trying dozens of different ways to write the world, hoping I could guess at what it wanted.

These two issues combined along with other similar issues meant that I ended up spending around 5-10% of the time I devoted to this project actually coding.

## Proposal tasks completion
Yes, I completed everything on my needs to have list. However, it is still worth mentioning that I took off two items. Those two items were hardware-related, and after realizing that this project was more complicated than expected due to the aforementioned issues, I took them off since they were not relevant to the computer science aspect of this project.

In terms of my nice to have list, I did make a bit of progress. The individual motors are error-corrected with PID. While this is just part of Webots, I did experiment with it to find which constants worked best for this specific case. However, I would not count this as a full completion of this task, since a full completion would include error correction of the whole robot by adding more sensors. Furthermore, I completed the 'remote control' task since the simulation is controlled through a completely different process by sending the instructions to a port, which are recieved by the simulation. I believe this is about as remote as the control can be for a simulation.

## Problems I couldn't solve
I had originally tried to use a different simulation: Gazebo. Looking back, this probably would have been a much better idea due to the faults of Webots, but there were some errors with dependencies or compatibility that I wasn't able to resolve.

## Advice to self
Due to having lots of activities, I ended up crashing out a bit over Christmas break, losing valuable time that I could have been working on my project. Going back, I would tell myself to continue to work on it a bit, even during lazy days, to avoid losing the flow of working.

I would also say to start with a less ambitious project and scale it up later rather than doing it the other way around. This would make the project a lot less overwhelming and would allow me to scale it up in a more appropriate way after I had done more research on the topic.