import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from PIL import Image, ImageTk #pip install pillow
from functools import partial

        


class firstpage(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        
       # load = Image.open("yoda.png")
       # photo = ImageTk.PhotoImage(load)
       # label = tk.Label(self, image = photo)
       # label.image = photo
       # label.place(x=0,y=0)

        border = tk.LabelFrame(self, text='Login', bg='ivory', bd = 10, font=("Arial", 20))
        border.pack(fill="both", expand = "yes", padx = 10, pady = 10)
        
        #Image code
        load = Image.open("yoda.png")
        photo = ImageTk.PhotoImage(load)
        label = tk.Label(border, image = photo)
        label.image = photo
        label.place(x=0,y=0)


        L1 = tk.Label(border,text="Username" , font =("Arial Bold", 15), bg='orange')
        L1.place(x=50, y=50)
        T1 = tk.Entry(border, width = 30, bd = 5)
        T1.place(x=180, y = 50)
        
        L2 = tk.Label(border,text="Password" , font =("Arial Bold", 15), bg = 'orange')
        L2.place(x=50, y=150)
        T2 = tk.Entry(border, width = 30, bd = 5, show = '*')
        T2.place(x=180, y = 150)
        
        def verify():
            if T1.get() == 'admin' and T2.get() == 'admin' : 
                controller.show_frame(secondpage)
                return
            try:
                with open("user_info.txt", "r") as f:
                    info = f.readlines() #makes list from file
                    i = 0
                    for e in info:
                        u, p = e.split(",") #username and password written and seperate by comma
                        if u.strip() == T1.get() and p.strip() == T2.get(): #removes extra spaces on txt file
                            controller.show_frame(thirdpage)
                            i = 1
                            break
                    if i == 0:
                        messagebox.showinfo("Error", "Please provide correct username and password")
            except:
                messagebox.showinfo("Error", "Please provide correct username and password")
            #else:
             #   messagebox.showinfo("Error", "Incorrect Username or Password")

        B1 = tk.Button(border, text="Login", font= ("Arial", 15), bg = "lime green", command = verify)
        B1.place(x=350, y=200)
        
        def register():
            window = tk.Tk()
            window.title("Register")    
            window.configure(bg = "deep sky blue")
            window.resizable(0,0) #makes sign-in window un-maximizable
            
            L1 = tk.Label(window, text = "Username:", font = ("Arial", 15), bg = "deep sky blue")
            L1.place(x=10, y=10)
            t1 = tk.Entry(window, width=30, bd=5) #bd = border
            t1.place(x=200,y=10)


            L2 = tk.Label(window, text = "Password:", font = ("Arial", 15), bg = "deep sky blue")
            L2.place(x=10, y=60)
            t2 = tk.Entry(window, width=30, bd=5, show = '*') #bd = border
            t2.place(x=200,y=60)
            
            L3 = tk.Label(window, text = "Confirm Password:", font = ("Arial", 15), bg = "deep sky blue")
            L3.place(x=10, y=110)
            t3 = tk.Entry(window, width=30, bd=5, show = '*') #bd = border
            t3.place(x=200,y=110)
            
            def check():
                if t1.get() != "" or t2.get() != "" or t3.get() != "": #checks for empty inputs
                    if t2.get() == t3.get(): #check if passwords match
                        with open("user_info.txt", "a") as f:
                            f.write(t1.get()+","+t2.get()+"\n")
                            messagebox.showinfo("Congrats!", "Welcome, you have successfully been registered!")
                    else:
                        messagebox.showinfo("Error", "Passwords do not match")
                else:
                    messagebox.showinfo("Error", "Please complete all required fields")
                        

            b1 = tk.Button(window, text= "Register", font=("Arial", 15), bg="#ffc22a", command = check) #yellow code
            b1.place(x=170, y=150)

            window.geometry("470x220")
            window.mainloop()
            

        B2 = tk.Button(border, text = "Register", bg = "lime green", font = ("Arial", 15), command = register)
        B2.place(x=200, y = 200)


class secondpage(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        
        border = tk.LabelFrame(self, text='Admin Page', bg='gainsboro', bd = 10, font=("Arial", 20))
        border.pack(fill="both", expand = "yes", padx = 10, pady = 10)
        

        load = Image.open("yoda.png")
        photo = ImageTk.PhotoImage(load)
        label = tk.Label(border, image = photo)
        label.image = photo
        label.place(x=0,y=50)
        
        modes = ["Autonomous" , "Manual"] 
        var = tk.StringVar(self)
        var.set(modes[0])
        drop_down = tk.OptionMenu(border, var, *modes)
        drop_down.pack()
      
        Label = tk.Label(border,text="Robot Mode:" , font =("Arial Bold", 20))
        Label.place(x=0, y=0)
        
        b1 = tk.Button(border, text = "Confirm mode", font = ("Arial", 15), bg = "lime green")
        b1.place(x=350, y = 0)

        Button = tk.Button(border, text="Delivery Page", font= ("Arial", 15), command = lambda: controller.show_frame(thirdpage))
        Button.place(x=0, y=250)
        
        Button = tk.Button(border, text="Logout", font= ("Arial", 15), command = lambda: controller.show_frame(firstpage))
        Button.place(x=0, y=300)




class thirdpage(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        
        border = tk.LabelFrame(self, text='Delivery Page', bg='gainsboro', bd = 10, font=("Arial", 20))
        border.pack(fill="both", expand = "yes", padx = 10, pady = 10)
        
        load = Image.open("yoda.png")
        photo = ImageTk.PhotoImage(load)
        label = tk.Label(border, image = photo)
        label.image = photo
        label.place(x=0,y=0)
        
        Label = tk.Label(border, text="Robot Status" , font =("Arial Bold", 20))
        Label.place(x=0, y=150)
        
        L1 = tk.Label(border,text="Status Input:" , font =("Arial Bold", 15))
        L1.place(x=0, y=200)
        T1 = tk.Entry(border, width = 30, bd = 5)
        T1.place(x=180, y = 200)
        
        def check_status():
            if T1.get() == "0": 
                
                Label = tk.Label(border, text="Ready for delivery" , font =("Arial Bold", 20))
                Label.place(x=250, y=150)
                
                return

            if T1.get() == "1":
            
                Label = tk.Label(border, text="      On route            " , font =("Arial Bold", 20))
                Label.place(x=250, y=150)
            
                return
            
            if T1.get() == "2":
            
                Label = tk.Label(border, text="Delivery complete" , font =("Arial Bold", 20))
                Label.place(x=250, y=150)
            
                return

            else:
                messagebox.showinfo("Error", "Incorrect input, Please enter 0, 1, or 2")

        B1 = tk.Button(border, text="Submit status", font= ("Arial", 15), command = check_status, bg='DarkOrange2')
        B1.place(x=0, y=250)
        
        buildings = ["Initial Building" , "KEC" , "Deerborn", "LINC", "MU"] 
        var = tk.StringVar(self)
        var.set(buildings[0])
        drop_down = tk.OptionMenu(border, var, *buildings)
        drop_down.pack()
      
        buildings1 = ["Destination Building" , "KEC" , "Deerborn", "LINC", "MU"] 
        var = tk.StringVar(self)
        var.set(buildings1[0])
        drop_down = tk.OptionMenu(border, var, *buildings1)
        drop_down.pack()
     
        Button1 = tk.Button(border, text = "Submit Delivery", font = ("Arial", 15), bg='DarkOrange2')
        Button1.place(x=200, y = 100)


        Button = tk.Button(border, text="Logout", font= ("Arial", 15), bg='DarkOrange2', command = lambda: controller.show_frame(firstpage))
        Button.place(x=0, y=300)


class Application(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        #creating a window
        window = tk.Frame(self)
        window.pack()
        
        window.grid_rowconfigure(0, minsize = 400) #800
        window.grid_columnconfigure(0, minsize = 600) #600

        self.frames = {}
        for F in (firstpage, secondpage, thirdpage):
            frame = F(window, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")
            
        self.show_frame(firstpage)
            
    def show_frame(self, page):
        frame = self.frames[page]
        frame.tkraise()
        self.title("GUI")

app = Application()
#app.maxsize(400,600)
app.mainloop()


