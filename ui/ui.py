import PySimpleGUI as sg

def get_input_layout():
    input_layout = [
        [sg.Frame(layout=[
            [sg.Multiline(size=(60, 10), key="scenario", default_text="While driving at night, a pedestrian is crossing the road from the front right."), ],
        ],
            title='Danger Driving Behavior', relief=sg.RELIEF_SUNKEN)],

        [sg.Button("Generate Scenario"), sg.Cancel()]
    ]
    return input_layout

if __name__ == '__main__':
    input_window = sg.Window('Customize Dangerous Driving Scenarios').Layout(get_input_layout())
    # Display and interact with the Window
    event, values = input_window.read()  # Part 4 - Event loop or Window.read call
    print(type(event))
    print(event, ':', values)
