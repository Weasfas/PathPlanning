package planExample;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

/**
 * @author David Yagüe Cuevas
 * @version 1.6
 * @Description A* Algorithm to solve a Path motion maze with n goals and 1 Source. Allowed Horizontal and vertical movement
 * as well as diagonal.
 * @Input File where the maze is constructed
 * @Output 2 Files: Solved maze with the heuristic required and 1 final file with execution statistics.
 * @since 18/10/2018
*/

public class AStarPlannerDiagonal {

	static CellD [][] grid = null; //Grid to store the maze; A Cell can be blocked: doing so its value will be set to null.
	private int height = 0;	//height of the matrix == number of rows.
	private int width  = 0; //width of the matrix == number of columns.
	private ArrayList<CellD> goals = new ArrayList<CellD>(); //List of goals.
	private ArrayList<CellD> path = new ArrayList<CellD>(); //Path given by the search.
	private ArrayList<Integer> distances = new ArrayList<Integer>(); //List of distances.
	private ArrayList<Integer> order = new ArrayList<Integer>(); //List to order the distances.
	/* Unbounded priority queue based on a priority heap
	 * provides O(log(n)) time for the enqueing and dequeing methods
	 * linear time for the remove and contains methods
	 * and constant time for the retrieval methods.
	 * */
	static PriorityQueue<CellD> open; //Fibonacci Heap to store open nodes.
	static boolean closed[][]; //Set of nodes already evaluated.
	static int startI, startJ; //Start position: row and column.
	static int endI, endJ; //End position: row and column.
	int costSoFar=0; //Total final cost of the path.
	int n_Expanded = 0; //Total number of nodes expanded.
	long execTime = 0; //Total execution time of the search.
	int n_goals = 0; //Total number of goals.
	String H_name; //Name of the heuristic A* is going to use.

	public ArrayList<Integer> getOrder(){
		return order;
	}
	
	public ArrayList<CellD> getPath(){
		return path;
	}
	
	public CellD [][] getGrid(){
		return grid;
	}
	
	public int getH(){
		return height;
	}
	
	public int getW(){
		return width;
	}
	
	/**
	 * @Description: AstartPlannerDiagonal constructor to solve the problem.
	 * @Param: fileName: Path of the file where the maze layout is, heuristic: Name of the heuristic to use.
	 * @Throws: IOException.
	 * @Calls: readFileAndSetDimensions(String), fillMazeAndPrepareSearch(String) and defineHeuristics(String).
	 */
	public AStarPlannerDiagonal(String fileName,String heuristic) throws IOException{
		readFileAndSetDimensions(fileName); //Read the file to set dimensions.
		fillMazeAndPrepareSearch(fileName); //Prepare the search.
		defineHeuristics(heuristic); //Define the heuristic to use.
	}

	/**
	 * @Description: Defines the heuristic within the maze passed as an argument.
	 * @Param: heuristic: Name of the heuristic to use.
	 * @throws IOException.
	 */
	public void defineHeuristics(String heuristic) throws IOException{
		int count=0;
		int index = 0;
		if(heuristic.equalsIgnoreCase("Manhattan")){ //If the heuristic is Manhattan distance.
			H_name="Manhattan distance.";
			//Loop the maze
			for(int i=0;i<height;++i){
				for(int j=0;j<width;++j){
					if(grid[i][j]!=null && grid[i][j].value=='G'){ //If the cell is a ghost its heuristic cost is set to 0.
						grid[i][j].heuristicCost=0;
					}else if(grid[i][j]!=null){ //Otherwise

						while(count<goals.size()){ //For each of the goals we have.
							CellD goal = goals.get(count); //Get the cell.
							int distance = Math.abs(i-goal.j)+Math.abs(j-goal.i); //Calculate the Manhattan distance.
							distances.add(distance); //Store the distance.
							count++;
						}
						int min = 0;
						//Loop to get the minimum distance in the set of distances we already filled.
						for (int a = 0; a < distances.size(); a++) {
							Integer f = distances.get(a);
							if (Integer.compare(f, min) < 0) {
								min = f;
								index = a;
							}
						}
						//update the heuristic cost of the cell. 
						grid[i][j].heuristicCost=distances.get(index);
						//Clear the set of distances so we can continue the loop.
						distances.clear();
						count=0;
					}

				}
			}

		}else if(heuristic.equalsIgnoreCase("Steven")){ //If the heuristic is Steven van Dijk one. 
			H_name="Steven van Dijk heuristic.";
			for(int i=0;i<height;++i){
				for(int j=0;j<width;++j){
					if(grid[i][j]!=null && grid[i][j].value=='G'){
						grid[i][j].heuristicCost=0;
					}else if(grid[i][j]!=null){
						int h = grid[i][j].heuristicCost;
						while(count<goals.size()){
							CellD goal = goals.get(count);
							int dx1 = i-goal.i;
							int dy1 = j-goal.j;
							int dx2 = startI-goal.i;
							int dy2 = startJ-goal.j;
							int cross = Math.abs(dx1*dy2 - dx2*dy1);
							h += cross;
							distances.add(h);
							count++;
						}
						int min = 0;
						for (int a = 0; a < distances.size(); a++) {
							Integer f = distances.get(a);
							if (Integer.compare(f, min) < 0) {
								min = f;
								index = a;
							}
						}
						grid[i][j].heuristicCost=distances.get(index);
						distances.clear();
						count=0;
					}

				}

			}
		}else if(heuristic.equalsIgnoreCase("Diagonal")){ //If the diagonal movement is allowed.
			H_name="Diagonal distance.";
			for(int i=0;i<height;++i){
				for(int j=0;j<width;++j){
					if(grid[i][j]!=null && grid[i][j].value=='G'){
						grid[i][j].heuristicCost=0;
					}else if(grid[i][j]!=null){
						int h = grid[i][j].heuristicCost;
						while(count<goals.size()){
							CellD goal = goals.get(count);
							int dx = Math.abs(i-goal.i);
							int dy = Math.abs(j-goal.j);
							if(dx>dy){
								h+=2*dy+1*(dx-dy);
							}else{
								h+=2*dx+1*(dy-dx);
							}
							distances.add(h);
							count++;
						}
						int min = 0;
						for (int a = 0; a < distances.size(); a++) {
							Integer f = distances.get(a);
							if (Integer.compare(f, min) < 0) {
								min = f;
								index = a;
							}
						}
						grid[i][j].heuristicCost=distances.get(index);
						distances.clear();
						count=0;
					}

				}

			}
		}else{
			System.err.println("Wrong heuristic.");
			throw new IOException();
		}

	}

	/**
	 * @Description: Read the maze layout to get the height and width.
	 * @Param: fileName: Path of the file where the maze layout is.
	 * @Throws: IOException.
	 */
	public void readFileAndSetDimensions(String filename) throws IOException {

		try{
			//Reader.
			BufferedReader read = new BufferedReader(new FileReader(filename));

			//Auxiliary variables to read the file.
			String readline;
			int rows = 0;
			char[] ch = null ;

			//Loop to read each line
			while((readline = read.readLine()) != null){
				//Read a line and store it in a character array.
				ch = readline.toCharArray();
				rows++;
			}

			//Set dimensions.
			this.width=ch.length;
			this.height=rows;

			//Close the reader.
			read.close();

		}catch(IOException e){
			//Error case --> File not found.
			System.err.println("Path of the file not correct");
		}
	}

	/**
	 * @Description: Fill the grid and prepare all for the search.
	 * @Param: fileName: Path of the file where the maze layout is.
	 * @Throws: IOException.
	 */
	public void fillMazeAndPrepareSearch (String filename) throws IOException{

		try{

			//Reader
			BufferedReader read2 = new BufferedReader(new FileReader(filename));

			//Auxiliary variables to read the file.
			String readline2;
			int num = 0;

			//Create the grid to fill and the closed set of cells to support the search.
			grid = new CellD[height][width];
			closed = new boolean[height][width];

			//Loop to read each line of the file.
			while((readline2 = read2.readLine()) != null){
				//Sore the line in the array
				char[] ch2 = readline2.toCharArray();
				//Loop to fill the grid depending on the value stored in the file.
				for(int i=0;i<ch2.length;++i){
					//Create the cell.
					grid[num][i] = new CellD(num, i);
					//Set the value attribute of the cell.
					if(ch2[i]=='S'){
						setStartCell(num, i);
						//If it is the start cell the final cost is always 0.
						grid[num][i].finalCost=0;
						grid[num][i].setAsSource();
					}else if (ch2[i]=='G'){
						goals.add(grid[num][i]);
						//Update the number of goals we have in the problem.
						n_goals++;
						grid[num][i].setAsGoal();
					}else if(ch2[i]=='%'){
						setBlocked(num, i);
					}else if(ch2[i]=='W'){
						grid[num][i].setAsWaypoint();
						//If it is a waypoint, its cost attribute must be changed to 2 instead of 4.
						grid[num][i].setCost();
					}
				}
				num++;
			}
			//Close the reader.
			read2.close();

		}catch(IOException e){
			//System.err.println("Error"); No need cause it is already printed.
		}
	}

	/**
	 * @Description: Method that performs the search.
	 * @Calls: Cell class setters, AStar(), printMaze() and reconstructPaths().
	 */
	public void doSearch(){

		//Control loop boolean.
		boolean seguir = true;
		//Auxiliary counters.
		int ii=0;
		int coint=0;
		//Set the fist goal.
		endI = goals.get(0).i;
		endJ = goals.get(0).j;

		//Loop until Robot has reached all goal points.
		while(seguir){
			//If there is more than one goal part of the maze must be reseted (Parents to begin the new search).
			if(n_goals>=2){	
				for(int i=0;i<height;++i){
					for(int j=0;j<width;++j){
						if(grid[i][j]!=null){
							grid[i][j].parent=null;
						}
					}
				}
			}

			//Reset closed and open set.
			closed = new boolean[height][width];
			open = new PriorityQueue<>((Object o1, Object o2) -> {
				CellD c1 = (CellD)o1;
				CellD c2 = (CellD)o2;

				return c1.finalCost<c2.finalCost?-1:
					c1.finalCost>c2.finalCost?1:0;
			});

			//If it is not the fist goal set the start cell with the end coordinates of the last iteration.
			if(coint!=0){
				setStartCell(endI,endJ);
			}

			//Update the counter
			coint++;

			//if there is more than one goal set the end coordinates to the next goal of the iteration.
			if(ii<n_goals){
				setEndCell(goals.get(ii).i,goals.get(ii).j);
			}

			ii++;

			//Perform A* search.
			AStar(); 

			//Trace back the path
			if(closed[endI][endJ]){

				CellD current = grid[endI][endJ];
				//Add the current node to the path list.
				path.add(current);
				while(current.parent!=null){					
					//Add each of the parents nodes to the path list.
					path.add(current.parent);
					//Set the current node to the next parent.
					current = current.parent;
				} 
				//Add the order of the "n" path.
				order.add(path.size());
				//Calculate the cost we have until now.
				costSoFar=grid[endI][endJ].finalCost;
				//If there is no path found:
			}else System.out.println("No possible path");
			//In the case we reached all goals, the loop stops.
			if(n_goals==coint){
				seguir=false;
			}

			//Calculate the nodes the search algorithm expanded.
			for(int i=0;i<height;++i){
				for(int j=0;j<width;++j){
					if(closed[i][j]){
						n_Expanded++;
					}
				}
			}

		}
	}

	/**
	 * @Description: Reconstruct the path A* found. This method is intended to debug, there is no real use in the final code since it is not called in any of the important methods.
	 */
	public void reconstructPaths(){
		System.out.println("\nPath:\n");
		for(int i=0;i<height;++i){
			for(int j=0;j<width;++j){
				if(grid[i][j]==null){
					System.out.print(" %");
				}else if (grid[i][j].value=='G' && path.contains(grid[i][j])){
					System.out.print(" " + goals.indexOf(grid[i][j]));
				}else if (grid[i][j].value=='S'){
					System.out.print(" S");
				}else if(path.contains(grid[i][j])){
					System.out.print(" X");
				}else{
					System.out.print("  ");
				}
			}
			System.out.println();
		}

		int w=0;
		int stop=0;
		System.out.println();
		while(w<n_goals){
			int index = order.get(w);

			for(int h=index-1;h>=stop;--h){
				if(w>0 && h!=stop) System.out.print(path.get(h-1)+" --> ");
				else if (w==0 && h>=stop)System.out.print(path.get(h)+" --> ");
			}
			w++;
			stop = index;
		}
		System.out.print("End\n");
		System.out.println("\nFinal cost:" + costSoFar);
	}

	/**
	 * @Description: Prints the maze layout. This method is intended to debug, there is no real use in the final code since it is not called in any of the important methods.
	 */
	public void printMaze(){
		System.out.println("\nMaze layout:\n");
		for(int i=0;i<height;++i){
			for(int j=0;j<width;++j){
				if(grid[i][j]==null){
					System.out.print(" %");
				}else if (grid[i][j].value=='G'){
					System.out.print(" G");
				}else if (grid[i][j].value=='S'){
					System.out.print(" S");
				}else if(grid[i][j].value=='W'){
					System.out.print(" W");
				}else{
					System.out.print("  ");
				}
			}
			System.out.println();
		}
	}

	/**
	 * @Description: Cell class, each position of the maze is a cell.
	 */
	static class CellD {  
		int heuristicCost = 0; //This is the heuristic cost of the cell.
		int finalCost = 0; //This is the evaluation function of the cell--> g + h.
		int i, j; //Variables to define the positions --> row(i), column(j).
		char value = ' '; //Specific value of the cell --> by default it is assumed to be blank space.
		int costMove = 4; //Cost of the movement to cell --> since it is assumed a black space, the cost of movement must be 4.	
		CellD parent; //Parent cell of the cell --> Use to reconstruct the path A* found.

		//Constructor - A cell is defined by its position within the maze.
		CellD(int i, int j){
			this.i = i;
			this.j = j;
		}

		//Useful to print the position of the cell.
		@Override
		public String toString(){
			return "["+this.i+", "+this.j+"]";
		}

		//Set the cell as goal --> Value = G.
		public void setAsGoal(){
			this.value='G';
		}

		//Set the cell as source --> Value = S.
		public void setAsSource(){
			this.value='S';
		}

		//Set the cell as waypoint --> Value = W.
		public void setAsWaypoint(){
			this.value='W';
		}

		/*Set the cost of the cell. By default all costs are initialized to 4 and when the maze is read and it is found 
		a position with a Waypoint, the cost of that position is update to 2.*/
		public void setCost(){
			this.costMove = 2;
		}

		//Define the way two Cell objects are the same.
		@Override
		public boolean equals(Object o){
			boolean same = false;
			if(o!=null && o instanceof CellD){
				boolean sameI = this.i == ((CellD) o).i;
				boolean sameJ = this.j == ((CellD) o).j;
				same = sameI && sameJ;
			}
			return same;
		}

	}

	//Block a Cell --> the value of the Cell is set to null.
	public static void setBlocked(int i, int j){
		grid[i][j] = null;
	}

	//Set a start Cell.
	public static void setStartCell(int i, int j){
		startI = i;
		startJ = j;
	}

	//Set a end Cell.
	public static void setEndCell(int i, int j){
		endI = i;
		endJ = j; 
	}

	//Method to check values and update costs if necessary.
	static void checkAndUpdateCost(CellD current, CellD t, int cost){
		if(t == null || closed[t.i][t.j])return; //If the cell is blocked or it is already closed --> nothing to update.
		int t_final_cost = t.heuristicCost+cost; //Update the Cell cost.

		boolean inOpen = open.contains(t); //Check if the Cell is already in the open list.
		if(!inOpen || t_final_cost<t.finalCost){ //If it is not || the estimation is upperbounded by the optimal cost.
			t.finalCost = t_final_cost; //Update its final cost
			t.parent = current; //Set the parent Cell
			if(!inOpen)open.add(t); //Add it to the open list
		}
	}

	//Method implementing A* algorithm.
	public static void AStar(){ 

		//add the start location to open list. This location correspond to the cell where the robot is located within the maze.
		open.add(grid[startI][startJ]);

		//Current cell.
		CellD current;

		//Endless loop
		while(!open.isEmpty()){ 
			//Retrieves and removes the head of the queue (returns null if it is empty).
			current = open.poll();
			//If it is null --> blocked cell, nothing to do here.
			if(current==null)break;
			//Close current Cell.
			closed[current.i][current.j]=true; 

			//If it is the goal --> End
			if(current.equals(grid[endI][endJ])){
				return; 
			}

			//Check neighbors
			CellD t;int cost=0;  
			if(current.i-1>=0){
				t = grid[current.i-1][current.j];
				if(t!=null)cost=t.costMove;
				checkAndUpdateCost(current, t, current.finalCost+cost); 

				//Allow diagonal movement
				if(current.j-1>=0){                      
					t = grid[current.i-1][current.j-1];
					if(t!=null)cost=t.costMove;
					checkAndUpdateCost(current, t, current.finalCost+cost); 
				}

				if(current.j+1<grid[0].length){
					t = grid[current.i-1][current.j+1];
					if(t!=null)cost=t.costMove;
					checkAndUpdateCost(current, t, current.finalCost+cost); 
				}
			} 

			if(current.j-1>=0){
				t = grid[current.i][current.j-1];
				if(t!=null)cost=t.costMove;
				checkAndUpdateCost(current, t, current.finalCost+cost); 
			}

			if(current.j+1<grid[0].length){
				t = grid[current.i][current.j+1];
				if(t!=null)cost=t.costMove;
				checkAndUpdateCost(current, t, current.finalCost+cost); 
			}

			if(current.i+1<grid.length){
				t = grid[current.i+1][current.j];
				if(t!=null)cost=t.costMove;
				checkAndUpdateCost(current, t, current.finalCost+cost); 

				//Allow diagonal movement
				if(current.j-1>=0){
					t = grid[current.i+1][current.j-1];
					if(t!=null)cost=t.costMove;
					checkAndUpdateCost(current, t, current.finalCost+cost); 
				}

				if(current.j+1<grid[0].length){
					t = grid[current.i+1][current.j+1];
					if(t!=null)cost=t.costMove;
					checkAndUpdateCost(current, t, current.finalCost+cost); 
				} 
			} 
		}
	}

	/**
	 * @Description: Write the output and statistics of the search in a file.
	 * @Param: completepath: Path of the file where the maze layout is.
	 */
	public void writeToFileWithCorrectFormat(String completepath){

		//Create to File objects.
		File actualFile = new File (completepath + ".output");
		File statisticFile = new File (completepath + ".statistics");

		try{
			//Create writer object with the statistics file.
			PrintWriter writer = new PrintWriter(statisticFile);
			//Write in the file the statistics of the search.
			writer.print("Statistics: \n\n");
			writer.print("Type of movement: Vertical, Horizontal and Diagonal.\n");
			writer.print("Heuristic used: " + H_name + "\n");
			writer.print("Execution time: " + execTime + " Milliseconds\n"); //Execution time.
			writer.print("Length of the path: " + path.size() + "\n"); //Length of the final path.
			writer.print("Number of nodes expanded: " + n_Expanded + "\n"); //# of nodes expanded.
			writer.print("Total cost: "+ costSoFar + "\n"); //Final total cost.

			//Path node by node.
			writer.print("Total final path:\n" );
			int w=0;
			int stop=0;
			while(w<n_goals){
				int index = order.get(w);
				for(int h=index-1;h>=stop;--h){
					if(w>0 && h!=stop) writer.print(path.get(h-1)+" --> ");
					else if (w==0 && h>=stop)writer.print(path.get(h)+" --> ");
				}
				w++;
				stop = index;
			}
			writer.print("End\n");
			//Close the writer.
			writer.close();

			//Create writer object with the mazes file.
			writer = new PrintWriter(actualFile);
			//Write maze layout.
			writer.println("Maze layout:\n");
			for(int i=0;i<grid.length;++i){
				for(int j=0;j<grid[0].length;++j){
					if(grid[i][j]==null){
						writer.print(" %");
					}else if (grid[i][j].value=='G'){
						writer.print(" G");
					}else if (grid[i][j].value=='S'){
						writer.print(" S");
					}else if(grid[i][j].value=='W'){
						writer.print(" W");
					}else{
						writer.print("  ");
					}
				}
				writer.println(" | ");
			}
			for(int a=0;a<grid[0].length+1;++a){
				writer.print(" -");
			}	
			writer.println();

			//Write the path follow to eat each of the ghosts
			int index=0;
			int index2=0;
			for(int control = 0;control<goals.size();++control){
				writer.println("\nPath to goal: " + control + "\n");
				index = order.get(control);
				for(int i=0;i<grid.length;++i){
					for(int j=0;j<grid[0].length;++j){
						if(grid[i][j]==null){
							writer.print(" %");
						}else if (grid[i][j].value=='G'){
							writer.print(" G");
						}else if (grid[i][j].value=='S' && control==0){
							writer.print(" S");
						}else if(path.subList(index2,index ).contains(grid[i][j])){
							writer.print(" X");
						}else{
							writer.print("  ");
						}
					}
					writer.println();
				}

				index2=index;
			}
			//Close the writer.
			writer.close();
		} catch (IOException e) {
			//Error case.
			System.err.println("Something went wrong. Try again");
		}

	}

	/*public static void main(String[] args) throws Exception{   

		try{

			AStarPlannerDiagonal b = new AStarPlannerDiagonal(args[0],args[1]);
			long start = System.currentTimeMillis();
			b.doSearch();
			long end = System.currentTimeMillis();
			execTime = end-start;
			b.writeToFileWithCorrectFormat(args[0]);
			System.out.print("Search finished.\nHeuristic used: " + args[1] + ".\nOutput files generated:\n" + 
					args[0] + ".output\n" + args[0] + ".statistics");
			
		}catch (IndexOutOfBoundsException r){
			//Nothing to do here
		}catch(Exception e){
			System.err.println("Bad argument. Try again.");
		}
	}*/
}