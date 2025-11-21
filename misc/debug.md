# Debugging Action Servers and Action Handler in the Context of Non-Twice Callability

## Decision Process
After trying various approaches from multiple LLMs and other sources, the decision was made to raise an action server and handler from ground zero and then add features incrementally until the unwanted behavior reappeared.

This approach was chosen because another action server was already working, and I prefer to process the logical steps in a **timer callback** to decouple it from other callbacks, ensuring deterministic timing and avoiding interference from other behaviors.

---

## Steps Taken
1. Created a new package and adapted all imports (including future ones) and added all necessary files.  
2. Wrote a **simple “count to ten” action server** and a client which calls it repeatedly.  
3. Used flags and function chains identical to those in the non-working program, but used the variable names from the working server, since the function chains were the same but the variable naming differed.  

> So far, the only change from the non-working server was the variable names.  
> If I could rebuild the non-working action server exactly like the working one, but just with the variable names of the working server, that would be the issue.

---

## Debugging at Rock Bottom
- Started only the action server and called the action goal manually, even twice or more. Everything worked fine.  
- By starting the action handler as well, everything also worked fine, and the goal was called repeatedly.  

At this stage, all other **listeners and publishers** were added, aiming to test on the robot to prove there is no race condition between node spinning and topic callbacks.

---

## Observations
- The issue seems **not to be the handler**, because manually firing a goal still works.  
- Removing the listeners did not fix the problem either, which is interesting because I thought this was the main difference.  
- Running the vision nodes locally (which always publish on the topics used) made it work.  

> Now the only difference is the presence of the topics `/odom` and `/cmd_vel`.  

- Even after letting the robot run so the topics exist, the problem persisted.  
- Topic existence was verified with `rqt`.

---

## Executor & Callback Groups
- Implemented a **MultiThreadedExecutor** and separate **Callback Groups** following ROS documentation recommendations to avoid potential deadlocks.  
- Replaced `spin_once` with the multi-threaded setup.

> This fixed the problem. It seems `spin_once` does not work properly on the ARM system.  
> Interestingly, this is not mentioned in any documentation but seems to be a known issue.
