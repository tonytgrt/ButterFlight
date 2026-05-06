

**Hi everyone,** today we are presenting ButterFlight, a dedicated authoring tool for realistic butterfly flight simulation. 

**The motivation** for this project comes from a common production problem: butterfly motion is difficult to animate manually because it is erratic, noisy, and unpredictable. **At the same time,** CFD-based simulation is usually too slow for production. **So our goal** is to create an efficient tool that generates believable butterfly flight while still giving artists control. 

**ButterFlight** supports several simulation modes. **For a single butterfly,** users can create hovering motion, free flight, or path-following motion along NURBS curves. **To make the movement** feel more natural, we also add procedural noise, so the butterfly does not move in a perfectly smooth way.  

**For swarm simulation, users** can simply toggle the swarm settings in the tool. **The system will** duplicate the selected butterfly and generate butterflies with flocking behaviors. This allows them to move as a group while still keeping individual variation, creating a more natural swarm effect. 

**We also include cinematic camera tools,** including a follow camera and a stationary rotating camera with auto-zoom. **This makes it easier** to preview and render the motion directly inside the tool. 

**For the technical foundation,** our approach is based on the SIGGRAPH 2022 paper “*A Practical Model for Realistic Butterfly Flight Simulation*.” **It uses three main components:** a parametric maneuvering function for wing-abdomen coupling, a force model combining quasi-steady aerodynamics with curl-noise vortex forces, and a maneuvering controller to guide the butterfly while keeping the motion realistic. 



**Next on to how to use our plugin for simulation. You would first need to load** up a rigged butterfly model in maya using the skeleton structure laid out here. **Then, open the butterflight** panel and select the root joint of the skeleton. **In hover mode** we can conveniently use the butterfly's current position and allow it to hover at that place. **We can also optionally** enable hovering noise for additional realism. **In output settings**, select the start frame and duration of the simulation for easy animation editing. **Then set the flap period** that determines how fast the butterfly will flap its wings. **Once everything's all set**, hit simulate to get an animation with all the specified settings.

**In path following mode,** we will be able to select a curve that the butterfly will follow and fly along. **We can either** make the butterfly start from the path start in the first frame, or use the current position and fly to the nearest point of the curve to start. **For the fly speed**, if we check use path speed scale, it will automatically calculate a speed that allows it to complete the entire path in the duration specified below. **Path noise is** also an option here for realism. **Deceleration and angle blend** are advanced settings if we want to blend the path following animation to a hovering pose later. **We can also create** a follow camera that stays at a fixed offset to the butterfly while it moves, or create a stationary camera that rotates itself and looks at the butterfly with an autozoom option. **All camera transformations** are captured using the current perspective camera for easy authorability.

**And finally we can enable swarm simulation.** It will create many agents of the butterfly and mimick the leader's move in a flocking behavior. This is supported in all our simulation modes as well.

**Let's end our presentation today** with a demo render that showcases what artists can use our tool to create. **In this first shot** we have the butterfly fly through this spiky terrain with a guided curve using a follow camera. **Path noise is enabled** here which can be better observed in the second overhead shot here. **In the third shot**, we used the deceleration and angle blend feature to allow the butterfly to smoothly blend into this hovering pose in front of the white rose. **And finally we used** the fly to nearest point feature to smoothly blend back to path following mode. **A stationary camera** is used as it watches the butterfly fly away to the flower field beyond.
