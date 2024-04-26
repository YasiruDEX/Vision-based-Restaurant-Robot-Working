import tkinter as tk

def table_clicked(table_number):
    print("Table", table_number, "clicked")

def main():
    root = tk.Tk()
    root.title("Restaurant Tables")

    canvas = tk.Canvas(root, width=800, height=800)
    canvas.pack()

    # Draw the grid
    for x in range(0, 800, 20):
        canvas.create_line(x, 0, x, 800, fill="lightgray", tags="grid")
    for y in range(0, 800, 20):
        canvas.create_line(0, y, 800, y, fill="lightgray", tags="grid")

    # Define the coordinates and size of each table
    table_positions = [
        (100, 100, 300, 300),   # Table 1
        (500, 100, 700, 300),   # Table 2
        (100, 500, 300, 700),   # Table 3
        (500, 500, 700, 700)    # Table 4
    ]

    for i, (x1, y1, x2, y2) in enumerate(table_positions, start=1):
        table = canvas.create_rectangle(x1, y1, x2, y2, fill="#00FF80")
        canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2, text="Table " + str(i), font=("", 20, "bold"), fill = "#000000")

        # Binding the click event to each table
        canvas.tag_bind(table, "<Button-1>", lambda event, table_number=i: table_clicked(table_number))

    root.mainloop()

if __name__ == "__main__":
    main()
