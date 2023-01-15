These are the 3 coding assignments I worked on during my artificial intelligence class. 
<br>The topics include uninformed and informed searches, constraint satisfication problems, and the resolution principle.
<br>*All coding is in the python language.
<br>*A pdf with instructions is included in pdf in each folder.

- Assignment 1 - Uninformed and informed searches
In this assigment we were given a pacman game where we had to use the premade instructions to make pacman go through a BFS, DFS, UCS, Greedy Best First Search, and A* Search.
<br>We also had to solve problems where pacman had to reach all four corners or all food pellets.
<br><br>*** Important Note ***
We worked on the files "search.py" and "search.agents.py" where ""*** YOUR CODE HERE ***"" is located.

  Command Line example - "python pacman.py -l testSearch -p AStarFoodSearchAgent"

- Assignment 2 - CSPs
In this assignment we try to find if a constraint problem can be satisfied. Our constraints come for a ".con" file and our variables and there domains come from a ".var" file. We check for both forward checking and no forward checking depending on what is put into the command line. Our answer is outputted into the command line. Answers for each our examples is in their own ".out" files.

  Command Line example - "python main.py ex1.var ex1.con none" or "python main.py ex1.var ex1.con fc"

- Assignment 3 - The Resolution Principle
In this assigment we use the resolution to prove that a clause if valid using the resolution principle given a knowledge base. This knowledge base is provided from a file with the extenstion ".kb". The steps and our answer is outputted into the console.

  Command Line example - "python main.py in.kb"
