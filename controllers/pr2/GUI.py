import tkinter as tk
import json


def gui(prompts:list,instructions:list):
    '''
    This gui is just for copy-pasting the instructions to ChatGPT. Once the connection is finished, this won't be needed anymore.
    '''
    result = {'user_input': None}
    # insert_point = 1
    for i in range(4):
        # Create the GUI window
        window = tk.Tk()
        window.title("ChatGPT Instructions")

        # Create a label to display what to do with the text
        label = tk.Label(window, text=instructions[i])
        label.pack()

        # Give the first prompt text
        prompt = tk.Text(window)#, width=50)
        prompt.insert('1.0', prompts[i])
        prompt.pack()

        # Close the GUI window after button is pushed
        def next():
            window.destroy()
        close_button = tk.Button(window, text="Next", command=next)
        close_button.pack()

        # run the mainloop
        window.mainloop()


    ###################################################################################


    # Create the GUI window
    window = tk.Tk()
    window.title("ChatGPT Instructions")

    # Create a label to display what to do with the text
    label = tk.Label(window, text='Instruction for ChatGPT: ')
    label.pack()

    # Create an entry widget for the user input
    entry = tk.Entry(window, width=50)
    entry.pack()
    def insert_text():
        result['user_input'] = entry.get()  # Store the value in the dictionary
        window.destroy()
    # Create a button widget to display the input
    display_button = tk.Button(window, text="Insert", command=insert_text)
    display_button.pack()

    # run the mainloop
    window.mainloop()


    ###################################################################################


    # Create the GUI window
    window = tk.Tk()
    window.title("ChatGPT Instructions")

    # Create a label to display what to do with the text
    label = tk.Label(window, text=instructions[5])
    label.pack()

    # Give the first prompt text
    prompt = tk.Text(window)#, width=50)
    prompt.insert('1.0', prompts[5]+'\n'+result['user_input']+'\n' + prompts[6])
    prompt.pack()

    # Close the GUI window after button is pushed
    def next():
        window.destroy()
    close_button = tk.Button(window, text="Next", command=next)
    close_button.pack()

    # run the mainloop
    window.mainloop()

def gui2(to_gui3 = False):
    result = {'user_input': None}
    def on_submit():
        result['user_input'] = entry.get()  # Store the value in the dictionary
        # Close the GUI window after submitting
        window.destroy()

    # Create the GUI window
    window = tk.Tk()
    window.title("ChatGPT Instructions")

    # Create a label to display the instruction
    label = tk.Label(window, text="If you want to change the instructions, chat with ChatGPT.\n" +
                                    "If you are satisfied, type in/paste the dictionary from ChatGPT's output:")
    label.pack()

    # Create a text widget to display the conversation
    display_text = tk.Text(window)#, height=30, width=150)
    display_text.pack()

    # Create an entry widget for the user input
    entry = tk.Entry(window, width=50)
    entry.pack()

    # Create a button to submit the input
    button = tk.Button(window, text="Submit", command=on_submit)
    button.pack()

    # Run the GUI event loop
    window.mainloop()

    if to_gui3:
        gui3(result['user_input'],from_gui2=True)
    else:
        return result['user_input']

    # return result['user_input']  # Return the value from the dictionary


def gui3(in_result:dict, from_gui2=False):
    if from_gui2:
        in_result = json.loads(in_result)
    result = {'user_input': in_result}
    start = False

    def redo():
        window.destroy()
        result['user_input'] = gui2(to_gui3=True)

    def start_sim():
        window.destroy()
        start = True

    def result_fun():
        res = result['user_input']["task_cohesion"]["step_instructions"]
        return res

    # Create the GUI window
    window = tk.Tk()
    window.title("ChatGPT Instructions")

    # Create a label to display what to do with the text

    label = tk.Label(window, text='The commands given by ChatGPT are:\n\n'+''.join([f'{i+1}:   {result_fun()[i]}\n' for i in range(len(result_fun()))]))
    label.pack()

    # Create a redo button
    redo_button = tk.Button(window, text="Redo", command=redo)
    redo_button.pack()

    # Create a start button
    start_button = tk.Button(window, text="Start Simulation", command=start_sim)
    start_button.pack()

    # Start the main loop
    window.mainloop()

    if start == True:
        return result['user_input']  # Return the value from the dictionary
    else:
        return in_result


def GUI(prompts:list,instructions:list):
    # chatGPT_response = gui(prompts,instructions)
    # chatGPT_response = json.loads(chatGPT_response)
    # gui3(chatGPT_response["task_cohesion"]["step_instructions"])

    gui(prompts,instructions)
    chatGPT_response = json.loads(gui2(to_gui3=False))
    chatGPT_response = gui3(chatGPT_response)

    return chatGPT_response

