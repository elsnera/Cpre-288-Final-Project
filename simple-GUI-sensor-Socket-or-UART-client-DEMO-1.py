# Author: Phillip Jones
# Date: 10/30/2023
# Description: Client starter code that combines: 1) Simple GUI, 2) creation of a thread for
#              running the Client socket in parallel with the GUI, and 3) Simple recieven of mock sensor 
#              data for a server/cybot.for collecting data from the cybot.



# Serial library:  https://pyserial.readthedocs.io/en/latest/shortintro.html 
import serial
import time # Time library   
# Socket library:  https://realpython.com/python-sockets/  
import socket
import tkinter as tk # Tkinter GUI library
# Thread library: https://www.geeksforgeeks.org/how-to-use-thread-in-tkinter-python/
import threading
import os  # import function for finding absolute path to this python script
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from tkinter import ttk
import msvcrt
import re
import timeit

# ##### START Define Functions  #########
# angle = [0, 64, 122]
# distance = [34,23,27]
# width2 = [62,20,12]
angle = []
distance = []
width2 = []
# linear_width = [12,2,25]
# final = zip(angle, distance, width, linear_width)
angle2 = []
distance2 =[]
clearance = []
clearanceWidth = []
for k in range (0,181):
        angle2.append(k)
        clearance.append(6)
        clearanceWidth.append(20)
        distance2.append(0)
global invoked 
invoked = 0

 




# Main: Mostly used for setting up, and starting the GUI
def main():
      

        global window  # Made global so quit function (send_quit) can access
        global gui_plot_message 
        global gui_uart_message
        global plot_button
        global invoked
        
        window = tk.Tk() # Create a Tk GUI Window
        window.geometry("1080x720")
        window.title = "GUI for CyBot"
        # Last command label  
        # global Last_command_Label  # Made global so that Client function (socket_thread) can modify
        # Last_command_Label = tk.Label(text="Last Command Sent: ") # Creat a Label
        # Last_command_Label.pack() # Pack the label into the window for display

        # gui_uart_message = tk.Label(text="Message Recieved From Server: ")
        
        # # Quit command Button
         # Pack the button into the window for display

        # # Cybot Scan command Button
        


        my_thread = threading.Thread(target=socket_thread) # Creat the thread

        my_thread.start() 
      
        # Start the thread
        # gui_plot_message = " "
        # if(gui_plot_message == "p\n"):
        larger_font = ("Helvetica", 16) 
        plot_button = ttk.Button(window, text="Reset Graph", command=graph_scan)
        plot_button.pack(side=tk.BOTTOM)
        

        quit_command_Button = tk.Button(text ="Press to Quit", command = send_quit)
        quit_command_Button.pack(side = tk.TOP)

        # scan_command_Button = tk.Button(text ="Press to Scan", command = send_scan)
        # scan_command_Button.pack(side=tk.BOTTOM) # Pack the button into the window for display

        gui_uart_message = tk.Label(text="Last UART Recieve: ", font= larger_font,  wraplength=300) # Creat a Label
        
        gui_uart_message.pack(pady=20)

        window.mainloop()
        
              
        # Start event loop so the GUI can detect events such as button clicks, key presses, etc.
       
# def theta_fill():
#       for k in range (0,181):
#         angle2.append(k)
#         clearance.append(6)
#         clearanceWidth.append(20)
      
def graph_scan():
        global invoked 
        global bars
        angle_degrees = np.arange(0,184,4)
        angle_radians = (np.pi/180) * angle_degrees
        global ax
        if hasattr(window, 'canvas'):
        # Clear the existing plot
                window.canvas.get_tk_widget().destroy()
        # angle2.clear()
        # clearance.clear()
        # clearanceWidth.clear()
        if(invoked == 0):
              if(len(distance2) == 181):
                    distance2.clear()
              for b in range (0,181):
                    distance2.append(0)
        if(invoked == 1):
                distance2.clear()
                invoked = 0
        
                x = 0
                z = 0
                if(len(distance2) != 180):
                        while(z <= 180):
                                if(x <= len(angle)-1):  
                                        if (angle2[z] == angle[x]):
                                                for j in range (width2[x]):
                                                        distance2.append(distance[x])
                                                        z+=1
                                                # if(x == len(angle)-1):
                                                #         break
                                                
                                                
                                                x+=1
                                        elif (angle2[z] != angle[x]):
                                                distance2.append(0)
                                                z+=1
                                else:
                                        z+=1
                                        distance2.append(0)
                        
                
        # y = len(distance2)          
        # for y in range(180-y):
        #         distance2.append(0)
                    


        #plt.pyplot.close()
        theta = np.radians(angle2)
        
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}) # One subplot of type polar
        
       
        #ax.plot(angle2, distance2, color='r', linewidth=4.0)  
         # Plot distance verse angle (in radians), using red, line width 4.0
        bars = ax.bar(theta, distance2, bottom=distance2, color='r', width= 0.020, alpha=0.75)
        bars = ax.bar(theta, clearance, bottom=0, color='b', width= 0.020, alpha=0.75)
        bars = ax.bar(theta, clearanceWidth, bottom=0, color='g', width=0.020, alpha=0.25)
       
        ax.set_xlabel('Distance (cm)', fontsize = 14.0)  # Label x axis
        ax.set_ylabel('Angle (degrees)', fontsize = 14.0) # Label y axis
        
        ax.tick_params(axis='both', which='major', labelsize=14) # set font size of tick labels
        if(distance != []):
         ax.set_rmax(max(distance) + 10) 
        else:
         ax.set_rmax(45) 
        #ax.set_rmin(3)                  # Saturate distance at 2.5 meters
       # ax.set_rticks([0.5,12])   # Set plot "distance" tick marks at .5, 1, 1.5, 2, and 2.5 meters
        ax.set_rlabel_position(-22.5)     # Adjust location of the radial labels
        ax.set_thetamax(180)              # Saturate angle to 180 degrees
        ax.set_xticks(np.arange(0,np.pi+.1,np.pi/4)) # Set plot "angle" tick marks to pi/4 radians (i.e., displayed at 45 degree) increments
                                             # Note: added .1 to pi to go just beyond pi (i.e., 180 degrees) so that 180 degrees is displayed
        ax.grid(True)                     # Show grid lines

# Create title for plot (font size = 14pt, y & pad controls title vertical location)
        ax.set_title("Mock-up Polar Plot of CyBot Sensor Scan from 0 to 180 Degrees", size=14, y=1.0, pad=-24) 
        canvas = FigureCanvasTkAgg(fig, master=window)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1) 
        window.canvas = canvas

def send_right():
        global gui_send_message # Command that the GUI has requested sent to the Cybot
        
        gui_send_message = "d\n" 
def send_back():
        global gui_send_message # Command that the GUI has requested sent to the Cybot
        
        gui_send_message = "s\n" 

#Send left command
def send_left():
        global gui_send_message # Command that the GUI has requested sent to the Cybot
        
        gui_send_message = "a\n" 
# Quit Button action.  Tells the client to send a Quit request to the Cybot, and exit the GUI
def send_quit():
        global gui_send_message # Command that the GUI has requested be sent to the Cybot
        global window  # Main GUI window
        
        gui_send_message = "quit\n"   # Update the message for the Client to send
        time.sleep(1)
        window.destroy() # Exit GUI


def get_user_input():
    user_input = ""
    while True:
        if msvcrt.kbhit():  # Check if a key is pressed
            char = msvcrt.getch().decode('utf-8')
            if char == '\r':  # Enter key
                break
            else:
                user_input += char
    return user_input


# Scan Button action.  Tells the client to send a scan request to the Cybot
def send_scan():
        global gui_send_message # Command that the GUI has requested sent to the Cybot       
        gui_send_message = "c\n"   # Update the message for the Client to send
        

def send_move():
        global gui_send_message
        gui_send_message = "w\n"

       
       


# Client socket code (Run by a thread created in main)
def socket_thread():
        theta = np.radians(angle2)
        # Define Globals
        global Last_command_Label # GUI label for displaying the last command sent to the Cybot
        global gui_send_message   # Command that the GUI has requested be sent to the Cybot
        global gui_uart_message
        global plot_button 
        global invoked
        start = 1
        




        # TCP Socket BEGIN (See Echo Client example): https://realpython.com/python-sockets/#echo-client-and-server
        HOST = "192.168.1.1"  # The server's hostname or IP address
        PORT = 288      # The port used by the server
        cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a socket object
        cybot_socket.connect((HOST, PORT))   # Connect to the socket  (Note: Server must first be running)
                      
        cybot = cybot_socket.makefile("rbw", buffering=0)  # makefile creates a file object out of a socket:  https://pythontic.com/modules/socket/makefile
        # TCP Socket END

       
        #send_message = "Hello\n"   # 2) Have user enter text
        gui_send_message = "wait\n"  # Initialize GUI command message to wait                                

         # Convert String to bytes (i.e., encode), and send data to the server
        
        #print("Sent to server: " + send_message) 
        send_message = ''

        # Send messges to server until user sends "quit"
        while send_message != 'quit\n':
                
                send_message = input("Enter a key: ")
                        #Parsing user input       
                try:
                        split = re.split('(\d+)', send_message)
                        send_message = split[0] + '\n'
                        times = int(split[1])
                        #new to final 1
                        
                except: 
                        times = 0
                      
                    

                print("SENT MESSAGE")
                #cybot.write(send_message.encode())
                # Update the GUI to display command being sent to the CyBot
                
        
                # Check if a sensor scan command has been sent
                if (send_message == "C\n") or (send_message == "c\n") or gui_send_message == "c\n":
                        distance2.clear()
                        distance.clear()
                        
                        cybot.write(send_message.encode())
                        
                        i = 0
                        print("Requested Sensor scan from Cybot:\n")
                        rx_message = bytearray(1) 
                        # Initialize a byte array
                        

                        # will need to add this command in main to send once all of the objects have been parsed
                        #also most likely need to wait a certain amount of seconds before recieving 
                       
                        
                                # Collect sensor data until "END" recieved
                        #UNDO UNTIL HERE
                        while(i < 89):
                                
                                rx_message = cybot.readline()   # Wait for sensor response, readline expects message to end with "\n"
                                i+=1
                                command_display = "Last UART Recieved: \t" + rx_message.decode()
                                
                                gui_uart_message.config(text = command_display)    
                                #grid is 7x11 by 13x11
                        rx_message = cybot.readline()
                        stop = rx_message
                        print(stop.decode())
                        check1 = 1
                        check2 = 0
                        while(stop != '\rOBJECT COMPLETE\n' ): 
                                a = 180
                                rx_message = cybot.readline()
                                stop = rx_message.decode()
                               
                                if(stop != '\rObject#\tAngle\tDistance (cm)\tWidth (deg)\tLinear Width (cm)\n' and check1 != 1 or (stop != '\rObject#\tAngle\tDistance (cm)\tWidth (deg)\tLinear Width (cm)\n' and check2 == 0) ):
                                        lastLine = stop.split(',')
                                        if(lastLine[0] != '\r180'):
                                         continue
                                        else:
                                 
                                            check1 = 1
                                elif(stop == '\rObject#\tAngle\tDistance (cm)\tWidth (deg)\tLinear Width (cm)\n'):
                                       print(stop)
                                       check1 = 1
                                       check2 = 1
                                       continue            
                                # elif(you == 0):
                                #        you = 1
                                #        continue             
                                elif(stop != '\rOBJECT COMPLETE\n' ):
                                 print(stop)
                                 line = stop.strip('\t').split(',')
                                 angle.append(int(line[1]))
                                 distance.append(int(line[2]))
                                 width2.append(int(line[3]))
                                 line.clear()
                                
                                # if(stop == '\rOBJECT COMPLETE\n'):
                                #         print(rx_message.decode())
                                
                                #         break
                        i = 0
                        invoked = 1
                        plot_button.invoke()
                        distance2.clear()
                        angle.clear()
                        width2.clear()
                        distance.clear()
                        
                
                        #gui_plot_message = "p\n"
                     
                       # file_object.close() # Important to close file once you are done with it!!                
                #Q turns left 45. C is scan, E turns right 45 degrees, X stops all movement, O key locks out
                
                elif send_message == "a\n" or send_message == "d\n" or  send_message == "s\n" or send_message == "w\n" or send_message == "e\n" or send_message == "q\n" or send_message == "o\n" or send_message == "x\n":
                        f = 0
                        
                        #message = ''
                        if(times != 0):
                                while(f < times):
                                        time.sleep(0.1)
                                        cybot.write(send_message.encode())
                                        message = cybot.readline()
                                        time.sleep(0.1)
                                        print(message.decode())
                                        command_display = "Last UART Recieved: \t" + message.decode()
                                        
                                        gui_uart_message.config(text = command_display) 
                                        f+=1
                                        
                                        if(message.decode().split()[0] == 'Obstacle'):
                                               break
                                       
                                        #if(send_message != 'o\n'):
                                                
                                      
                        else:
                                
                                cybot.write(send_message.encode())    
                               # if(send_message != 'o\n'):
                                message = cybot.readline()
                                
                                time.sleep(0.1)
                                print(message.decode())
                                command_display = "Last UART Recieved: \t" + message.decode()
                                gui_uart_message.config(text = command_display)
                        

                         
                else:
                        print("NOT A COMMAND")
                        continue 

               

                # else:                
                #         print("Waiting for server reply\n")
                #         rx_message = cybot.readline()      # Wait for a message, readline expects message to end with "\n"
                #         print("Got a message from server: " + rx_message.decode() + "\n") # Convert message from bytes to String (i.e., decode)

              
                # while gui_send_message == "wait\n": 
                #         time.sleep(.1)  # Sleep for .1 seconds
                #send_message = gui_send_message
                #else:
                #        send_message = gui_send_message  # GUI has requested a new command
                
               
                     
                
                        #result = get_user_input() + '\n'
                        # gui_uart_message = cybot.readline()
                        #  # Reset gui command message request to wait                        
                
                #cybot.write(send_message.encode()) 

        print("Client exiting, and closing file descriptor, and/or network socket\n")
        time.sleep(2) # Sleep for 2 seconds
        cybot.close() # Close file object associated with the socket or UART
        cybot_socket.close()  # Close the socket (NOTE: comment out if using UART interface, only use for network socket option)



### Run main ###
main()
