from future import __division__
import math

objects = [1, 2, 3, 4, 5]

# state_h, a tuple of two objects, what the human wants, and what the robot believes (is the item the human wants)
# state_r, which object the human wants

state_h = (objects[0], [1/len(objects) for o in objects]) # simplification of the distribution over objects
state_r = [1/len(objects) for o in objects]

# actions, a map from possible actions to their affect on the 
# distributions
actions = { 1 : lambda x: x, 
        }


b_r = [1/len(objects) for o in objects]
# assume the human thinks we think anything is possible
b_h = [1/len(objects) for o in objects]

while True: 
    candidate_b_hs = [(action, bayes_filter_update(b_h, action)) for action in actions]
    # we want the minimum information lost when we use the robot's belief
    # to approximate the human's belief
    # KL divergence : the information lost when using Q to approximate P
    kl_divergence = lambda P, Q: sum([p * math.log(p/q, 2) for p, q in zip(P, Q)])
    best_action, resulting_b_h = min(candidate_b_hs, key=lambda x: kl_divergence(x[1], b_r))
    b_h = resulting_b_h
    b_r = bayes_filter_update(b_r, get_observation())




        


