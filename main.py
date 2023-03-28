from tkinter import *
import time

U_WINDOW_HEIGHT = 700
U_WINDOW_WIDTH = 1080
TEXT = ("LPM - ustawienie przeszkody\nPPM - usunięcie przeszkody\n"
        "CTRL + LPM - ustawienie startu\nCTRL + PPM - ustawienie mety")

class Field:
    def __init__(self, view, field_type):
        self.view = view                #obiekt wyświetlany na ekranie
        self.field_type = field_type    #typ pola których są 4: empty, obstacle, start i finish
        self.g = 0                      #odległośc od punktu startowego do aktualnej pozycji
        self.h = 0                      #najkrótsza ścieżka z aktualnej pozycji do mety
        self.f = 0                      #suma g i h
        self.came_from = []             #poprzednie pole na trasie

class Settings:
    def __init__(self):
        
        self.fields = []                #dwuwymiarowa lista przechowująca obiekty klasy Field
        self.number_of_fields = 0       #liczba pól w rzędzie i kolumnie
        self.fields_size = 0            
        self.is_start_set = False       
        self.is_finish_set = False      
        self.start_coords = []          
        self.finish_coords = []         

    def set_fields(self, number_of_fields, canvas):
        
        for x in self.fields:
            x.clear()
        self.fields.clear()
        canvas.delete('all')   

        self.number_of_fields = number_of_fields
        self.fields_size = int(U_WINDOW_HEIGHT / self.number_of_fields)
        self.is_start_set = False
        self.is_finish_set = False


        for x in range(self.number_of_fields):
            self.fields.append([])
            x_coord = self.fields_size * x

            for y in range(self.number_of_fields):
                y_coord = self.fields_size * y
                field_rect = canvas.create_rectangle(x_coord, y_coord, x_coord + self.fields_size, 
                           y_coord + self.fields_size, fill="white")
                
                field = Field(field_rect, "empty")
                self.fields[x].append(field)
    
    def count_field(self, x, y):
        #na podstawie pozycji kursora liczy które pole zostało kliknięte
        x = int(x / self.fields_size)
        y = int(y / self.fields_size)
        return [x, y]

    def set_obstacle(self, event, canvas):
        x, y = self.count_field(event.x, event.y)

        if self.fields[x][y].field_type == "start":
            self.is_start_set = False
        elif self.fields[x][y].field_type == "finish":
            self.is_finish_set = False

        canvas.itemconfig(self.fields[x][y].view, fill="black")
        self.fields[x][y].field_type = "obstacle"

    def delete_obstacle(self, event, canvas):
        x, y = self.count_field(event.x, event.y)

        if self.fields[x][y].field_type == "start":
            self.is_start_set = False
        elif self.fields[x][y].field_type == "finish":
            self.is_finish_set = False

        canvas.itemconfig(self.fields[x][y].view, fill="white")
        self.fields[x][y].field_type = "empty"

    def set_start(self, event, canvas):
        if self.is_start_set:
            return

        x, y = self.count_field(event.x, event.y)
        self.is_start_set = True
        self.start_coords = [x, y]

        canvas.itemconfig(self.fields[x][y].view, fill="green")
        self.fields[x][y].field_type = "start"

    def set_finish(self, event, canvas):
        if self.is_finish_set:
            return

        x, y = self.count_field(event.x, event.y)
        self.is_finish_set = True
        self.finish_coords = [x, y]

        canvas.itemconfig(self.fields[x][y].view, fill="red")
        self.fields[x][y].field_type = "finish"

    def reset(self, canvas, astar):
        for rows in self.fields:
            for element in rows:
                element.g = 0
                element.h = 0
                element.f = 0
                element.came_from = []

                astar.openset.clear()
                astar.closedset.clear()

                if element.field_type == "empty":
                    canvas.itemconfig(element.view, fill="white")

class Astar:
    def __init__(self):
        self.openset = []       #lista pól nieodwiedzonych sąsiadujących z odwiedzonymi
        self.closedset = []     #lista pól odwiedzonych

    def find_path(self, settings, canvas, is_show_searching, window):
        #główna funkcja algorytmu

        self.openset.clear()
        self.closedset.clear()

        #sprawdza czy start i finish zostały ustawione
        if not settings.is_start_set or not settings.is_finish_set:
            return

        #pierwszym polem trafiającym do listy pól nieodwiedzonych jest pole startowe
        self.openset.append(settings.start_coords)
        current_x, current_y = settings.start_coords

        #przypisywane mu są wartości g, h i f
        settings.fields[current_x][current_y].g = 0
        
        #główna pętla algorytmu
        while len(self.openset) != 0:
            if is_show_searching.get():
                window.update()
                time.sleep(0.05)

            #w liście pól nieodwiedzonych szukany jest wierzchołek z najmniejszą wartością f
            current_field = self.find_lowest_f_value(settings)
            current_x, current_y = current_field

            #jeżeli wierzchołek jest metą to zakończ pętlę
            if settings.fields[current_x][current_y].field_type == "finish":
                self.reconstruct_path(settings, canvas, window, is_show_searching)
                break
           
            self.closedset.append(current_field)
            neighbour_fields = self.find_neighbours(current_field, settings, canvas)
            for neighbour in neighbour_fields:
                neighbour_x, neighbour_y = neighbour

                #jeżeli pole zostało już odwiedzone zostaje pominięte
                if self.is_in_list(neighbour, self.closedset):
                    continue

                #oblicza długość ścieżki od startu do sąsiada aktualnie odwiedzanego wierzchołka
                tentative_g_score = settings.fields[current_x][current_y].g + self.count_distance(current_field, neighbour)
                tentative_is_beter = False  #flaga informująca czy znaleziono krótszą trasę

                #sprawdza czy obliczona trasa do sąsiada jest najkrótsza 
                if not self.is_in_list(neighbour, self.openset):
                    #jeżeli pole nie sąsiadowało z żadnym odwiedzanym wcześniej polem
                    #to znaczy że obliczona trasa jest najkrótsza
                    self.openset.append(neighbour)
                    settings.fields[neighbour_x][neighbour_y].h = self.count_distance(neighbour, settings.finish_coords)
                    tentative_is_beter = True
                elif tentative_g_score < settings.fields[neighbour_x][neighbour_y].g:
                    #jeżeli obliczona długość trasy jest krótsza niż ta którą pole już posiadało
                    #flaga o znalezieniu krótszej trasy zostaje ustawiona na true
                    tentative_is_beter = True
                
                #jeżeli znaleziono najkrótszą scieżkę do sąsiedniego wierzchołka
                #następuje przypisanie wartości
                if tentative_is_beter:
                    settings.fields[neighbour_x][neighbour_y].came_from = current_field
                    settings.fields[neighbour_x][neighbour_y].g = tentative_g_score
                    settings.fields[neighbour_x][neighbour_y].f = (settings.fields[neighbour_x][neighbour_y].g 
                                                                   + settings.fields[neighbour_x][neighbour_y].h)
        return

    def reconstruct_path(self, settings, canvas, window, is_show_searching):
        coords = settings.finish_coords

        while True:
            coords = settings.fields[coords[0]][coords[1]].came_from
            if settings.fields[coords[0]][coords[1]].field_type == "start":
                break
            canvas.itemconfig(settings.fields[coords[0]][coords[1]].view, fill="cyan")
            if is_show_searching.get():
                window.update()
                time.sleep(0.05)

    def is_in_list(self, element, list):
        for x in list:
            if element == x:
                return 1
        return 0

    def find_neighbours(self, current_coords, settings, canvas):
        neighbours = []     #lista sąsiadujących pól

        x = -1
        while x <= 1:
            if current_coords[0] + x < 0 or current_coords[0] + x > settings.number_of_fields - 1:
                #jeżeli pole znajduje się przy skrajnie prawej lub lewej krawędzi
                #pomija pola wychodzące poza ekran
                x += 1
                continue
            
            y = -1
            while y <= 1:
                if current_coords[1] + y < 0 or current_coords[1] + y > settings.number_of_fields - 1:
                    #analogiczna sytuacja w osi y
                    y += 1
                    continue
                elif x == 0 and y == 0:
                    #pomija pole dla którego wyszukiwani są sąsiedzi
                    y += 1
                    continue
                if settings.fields[current_coords[0] + x][current_coords[1] + y].field_type == "obstacle":
                    #pomija przeszkody
                    y += 1
                    continue
                elif settings.fields[current_coords[0] + x][current_coords[1] + y].field_type == "empty":
                    #jeżeli pole jest puste to zostaje pokolorowane
                    #chodzi o to żeby nie kolorowało startu i mety
                    canvas.itemconfig(settings.fields[current_coords[0] + x][current_coords[1] + y].view, fill="orange")
                neighbours.append([current_coords[0] + x, current_coords[1] + y])
                    
                y += 1
            x += 1
        return neighbours

    def find_lowest_f_value(self, settings):
        #funkcja zwraca koordynaty na pole z najmniejszą wartością f

        lowest_f_value = settings.fields[self.openset[0][0]][self.openset[0][1]].f  
        lowest_f_field = self.openset[0]    #koordynaty pola z najmniejszą wartością f
        element_index = 0   #indeks elementu z najmniejszą wartością f

        for element in reversed(range(len(self.openset))):
            element_f_value = settings.fields[self.openset[element][0]][self.openset[element][1]].f
            if element_f_value < lowest_f_value:
                lowest_f_value = element_f_value
                lowest_f_field = self.openset[element]
                element_index = element
        
        #usuwa element z najmniejszą wartością f z listy nieodwiedzonych wierzchołków
        self.openset.pop(element_index)
        return lowest_f_field

    def count_distance(self, start, finish):
        #funkcja liczy dystans z pola "start" do pola "finish"
        #długośc jednego pola w lini prostej ma wartość 10
        #długość jednego pola w ukosie ma wartość 14

        distance = 0
        distance_x = abs(start[0] - finish[0])
        distance_y = abs(start[1] - finish[1])


        straight_distance = abs(distance_x - distance_y)
        distance = 10 * straight_distance
        if distance_x > distance_y:
            distance += 14 * (distance_x - straight_distance)
        else:
            distance += 14 * (distance_y - straight_distance)
        return distance


def main():

    root = Tk()
    settings = Settings()
    astar = Astar()

    #tworzenie okna
    root.title("A*")
    root.geometry(str(U_WINDOW_WIDTH) + "x" + str(U_WINDOW_HEIGHT))
    root.resizable(FALSE, FALSE)
    

    #tworzenie widgetów
    is_show_searching = IntVar()
    canvas = Canvas(root, width=str(U_WINDOW_HEIGHT), height=str(U_WINDOW_HEIGHT), bd=-2, highlightbackground='black')
    slider = Scale(root, from_=10, to=40, orient=HORIZONTAL, length=200, \
           command=lambda a: settings.set_fields(slider.get(), canvas))
    show_searching = Checkbutton(root, text = "Show Searching", variable=is_show_searching)
    start_button = Button(root, text="Start", font="Arial, 20", 
                 command=lambda: astar.find_path(settings, canvas, is_show_searching, root))
    reset_button = Button(root, text="Reset", font="Arial, 20", command=lambda: settings.reset(canvas, astar))
    text = Label(root, text=TEXT)
    slider.set(20)


    #obsługa myszy
    canvas.bind('<B1-Motion>', lambda event: settings.set_obstacle(event, canvas))
    canvas.bind('<Button-1>', lambda event: settings.set_obstacle(event, canvas))

    canvas.bind('<B3-Motion>', lambda event: settings.delete_obstacle(event, canvas))
    canvas.bind('<Button-3>', lambda event: settings.delete_obstacle(event, canvas))

    canvas.bind('<Control-Button-1>', lambda event: settings.set_start(event, canvas))
    canvas.bind('<Control-Button-3>', lambda event: settings.set_finish(event, canvas))

    #pozycjonowanie elementów
    canvas.grid(row = 0, column = 0, rowspan = 5)
    slider.grid(column = 1, row = 0, padx=(50))
    show_searching.grid(column=1, row = 1)
    text.grid(column=1, row=2)
    start_button.grid(column = 1, row = 3)
    reset_button.grid(column = 1, row = 4)  
    

    settings.set_fields(slider.get(), canvas)   
    root.mainloop()


if __name__ == "__main__":
    main()