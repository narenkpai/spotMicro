import os
import subprocess

# List of Python scripts in the folder
programs = ["stand.py", "walkingGait.py", "sit.py"]  # Replace with actual filenames

def run_program():
    while True:
        print("\nSelect a program to run:")
        for i, program in enumerate(programs, 1):
            print(f"{i}. {program}")
        print("q. Quit")

        choice = input("Enter the number of the program to run (or 'q' to quit): ").strip().lower()

        if choice == "q":
            print("Exiting program. Goodbye!")
            break

        try:
            choice = int(choice)
            if 1 <= choice <= len(programs):
                selected_program = programs[choice - 1]
                print(f"Running {selected_program}...\n")
                subprocess.run(["python", selected_program], check=True)
            else:
                print(f"Invalid selection. Please enter a number between 1 and {len(programs)} or 'q' to quit.")
        except ValueError:
            print("Invalid input. Please enter a valid number or 'q' to quit.")

if __name__ == "__main__":
    run_program()
