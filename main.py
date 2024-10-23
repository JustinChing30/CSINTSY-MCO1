import DFS
import GBFS
import A_star

def main():
    while True:  # Loop to allow repeated algorithm selection
        print("\nSelect the algorithm to use:")
        print("1. Depth-First Search (DFS)")
        print("2. Greedy Best-First Search (GBFS))")
        print("3. A* Search")
        print("4. Exit")  # Option to exit the program

        choice = input("\nEnter the number of your choice: ")

        if choice == '1':
            DFS.main()  # Call the main function of dfs.py
        elif choice == '2':
            GBFS.main()  # Call the main function of gbfs.py
        elif choice == '3':
            A_star.main()  # Call the main function of a_star.py
        elif choice == '4':
            print("Exiting the program...")
            break  # Exit the loop and end the program
        else:
            print("Invalid choice. Please select 1, 2, 3, or 4.")


if __name__ == '__main__':
    main()