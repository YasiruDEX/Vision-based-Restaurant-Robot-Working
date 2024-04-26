import tkinter as tk
import math
from heapq import heappop, heappush
import random
from gtts import gTTS
import os
import pygame
import time

# Define the centers of each table
table_locations = [(200, 200), (600, 200), (200, 600), (600, 600), (1000, 200), (1400, 200), (1000, 600), (1400, 600)]

table_points = []
destination_points = []

COMMAND_FULL = ""

active_location = (200,320)

def text_to_speech(text, lang='en'):
    tts = gTTS(text=text, lang=lang)
    tts.save("C:/Users/yasir/Downloads/output.mp3")
    pygame.init()
    pygame.mixer.init()
    pygame.mixer.music.load("C:/Users/yasir/Downloads/output.mp3")
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)
    pygame.mixer.music.stop()  # Stop the music
    pygame.mixer.quit()  # Unload the music

def play_welcome_phrase():
    welcome_text = "Welcome to the restaurant. Please select a table. To deliver the order"
    text_to_speech(" Hi ")
    text_to_speech(welcome_text)


def table_clicked(x, y, canvas):
    global COMMAND_FULL
    global active_location
    print("Clicked at", x, y)
    print("Table", str(active_location), "is active")
    print("Table",str(active_location), "coordinates:", active_location)

    canvas.create_oval(active_location[0] - 6, active_location[1] - 6, active_location[0] + 6, active_location[1] + 6, fill="cyan")

    # Calculate shortest path
    path = find_shortest_path((x, y), (active_location[0], active_location[1]))
    print("Shortest path:", path)
    draw_path(path, canvas)  # Pass canvas to draw_path
    COMMAND = ""
    if len(path) == 2:
        COMMAND = COMMAND + str(abs(path[0][0] - path[1][0])) + " "
    else:
        COMMAND = COMMAND + str(abs(path[0][0] - path[1][0])) + " "
        if path[0][0] < path[2][0]:
            if path[1][1] < path[2][1]:
                COMMAND = COMMAND + str("R") + " "
            else:
                COMMAND = COMMAND + str("L") + " "
        else:
            if path[1][1] < path[2][1]:
                COMMAND = COMMAND + str("L") + " "
            else:
                COMMAND = COMMAND + str("R") + " "
        COMMAND = COMMAND + str(abs(path[1][1] - path[2][1])) + " "
        if path[1][1] < path[3][1]:
            if path[2][0] < path[3][0]:
                COMMAND = COMMAND + str("L") + " "
            else:
                COMMAND = COMMAND + str("R") + " "
        else:
            if path[2][0] < path[3][0]:
                COMMAND = COMMAND + str("R") + " "
            else:
                COMMAND = COMMAND + str("L") + " "
        COMMAND = COMMAND + str(abs(path[2][0] - path[3][0])) + " "

    print("Command:", COMMAND)
    COMMAND_FULL = "COMMAND: " + COMMAND

    canvas.create_rectangle(0,20,1600,60, fill="white")
    canvas.create_text(800,40, text = COMMAND_FULL, font=("Arial", 20, "bold"), fill="#000000")

    text_to_speech(COMMAND_FULL)
    
    active_location = [x,y]

def heuristic(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def find_shortest_path(goal, start):
   path = []
   path.append(start)
   up_down = True
   if start[1] == goal[1]:
       up_down = False

   left_pane = False

   middle_pane = (start[0] + goal[0])//2

   if goal[0] < middle_pane:
       left_pane = True

   if up_down:
       if not left_pane:
           path.append((goal[0] - 200, start[1]))
           path.append((goal[0] - 200, goal[1]))
           path.append(goal)
       else:
           path.append((goal[0] + 200, start[1]))
           path.append((goal[0] + 200, goal[1]))
           path.append(goal)
   else:
         path.append((goal[0], goal[1]))
   
   return path

def draw_path(path, canvas):  # Pass canvas to draw_path
    path_color = random.choice(["red", "green", "blue", "yellow", "purple", "orange", "cyan", "magenta", "brown", "pink"])
    if path:
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            canvas.create_line(x1, y1, x2, y2, fill=path_color, width=4)
            time.sleep(1)

def main():
    global table_points, destination_points, COMMAND_FULL
    root = tk.Tk()
    root.title("Restaurant Tables")

    canvas = tk.Canvas(root, width=1600, height=800)
    canvas.pack()

    # Draw the grid
    for x in range(0, 1600, 20):
        canvas.create_line(x, 0, x, 1600, fill="lightgray", tags="grid")
    for y in range(0, 800, 20):
        canvas.create_line(0, y, 1600, y, fill="lightgray", tags="grid")

    table_points = []  # Clear the table points array
    destination_points = []  # Clear the destination points array

    # Use table_locations array to define the coordinates and size of each table
    for i, (center_x, center_y) in enumerate(table_locations, start=1):
        table_size = 200  # Define the size of each table
        x1 = center_x - table_size // 2
        y1 = center_y - table_size // 2
        x2 = center_x + table_size // 2
        y2 = center_y + table_size // 2
        table = canvas.create_rectangle(x1, y1, x2, y2, fill="#00FF80")
        canvas.create_text(center_x, center_y, text="Table " + str(i), font=("", 20, "bold"), fill="#000000")

        # Draw blue dot under 1 grid point from each table
        dot_x = center_x
        dot_y = center_y + table_size // 2 + 20  # Offset by one grid point downwards
        if i == active_location:
            dot_color = "cyan"
        else:
            dot_color = "blue"
        canvas.create_oval(dot_x - 6, dot_y - 6, dot_x + 6, dot_y + 6, fill=dot_color)
        destination_points.append((dot_x, dot_y))

        # Binding the click event to each table
        canvas.tag_bind(table, "<Button-1>", lambda event, x=center_x, y=center_y: table_clicked(x, y + table_size // 2 + 20, canvas))

        # Calculate and add points inside the table to table_points array
        for x in range(x1, x2+20, 20):
            for y in range(y1, y2 + 20, 20):
                table_points.append((x, y))

        for point in table_points:
            canvas.create_rectangle(point[0]-1, point[1]-1, point[0] + 1, point[1] + 1, fill="black")

    root.after(100, play_welcome_phrase)

    root.mainloop()

if __name__ == "__main__":
    main()
