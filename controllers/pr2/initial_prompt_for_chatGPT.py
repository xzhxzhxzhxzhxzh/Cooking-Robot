import json
from GUI import GUI
# from chatGPT_interaction import chatgpt_call

def env_example(basic=True):
    cyber_burger_king = {   'tables':   {   'ingredients_table':    {   'bread':6,
                                                                        'lettuce':6,
                                                                        'beef':6,
                                                                        'cheese':6,
                                                                        'tomato':6},
                                            'serving_table':        {   'bread':0,
                                                                        'lettuce':0,
                                                                        'beef':0,
                                                                        'cheese':0,
                                                                        'tomato':0}},
                            'ingredients_list': ['bread','lettuce','beef','cheese','tomato']}

    if basic == False:
        cyber_burger_king['tables']['ingredients_table']['bread'] -= 2
        cyber_burger_king['tables']['ingredients_table']['lettuce'] -= 1
        cyber_burger_king['tables']['ingredients_table']['beef'] -= 1
        cyber_burger_king['tables']['ingredients_table']['cheese'] -= 1
        cyber_burger_king['tables']['ingredients_table']['tomato'] -= 1
        cyber_burger_king['tables']['serving_table']['bread'] += 2
        cyber_burger_king['tables']['serving_table']['lettuce'] += 1
        cyber_burger_king['tables']['serving_table']['beef'] += 1
        cyber_burger_king['tables']['serving_table']['cheese'] += 1
        cyber_burger_king['tables']['serving_table']['tomato'] += 1

    return cyber_burger_king


def output_example():
    out_ex = {"task_cohesion": {    "task_sequence": [
                    "take(target_hand=\'left\', target_obj=\'bread\')",
                    "take(target_hand=\'right\', target_obj=\'lettuce\')",
                    "rotate_180()",
                    "put(target_hand=\'left\')",
                    "put(target_hand=\'right\')",
                    "rotate_180()",
                    "take(target_hand=\'left\', target_obj=\'beef\')",
                    "take(target_hand=\'right\', target_obj=\'cheese\')",
                    "rotate_180()",
                    "put(target_hand=\'left\')",
                    "put(target_hand=\'right\')",
                    "rotate_180()",
                    "take(target_hand=\'left\', target_obj=\'tomato\')",
                    "take(target_hand=\'right\', target_obj=\'bread\')",
                    "rotate_180()",
                    "put(target_hand=\'left\')",
                    "put(target_hand=\'right\')",
                    ],

                    "step_instructions": [
                    "take a piece of bread from the ingredients_table with the left hand",
                    "take a piece of lettuce from the ingredients_table with the right hand",
                    "turn around to face the serving_table",
                    "put down the bread in the left hand on the serving_table",
                    "put down the lettuce in the right hand on the serving_table",
                    "turn around to face the ingredients_table",
                    "take a piece of beef from the ingredients_table with the left hand",
                    "take a piece of cheese from the ingredients_table with the right hand",
                    "turn around to face the serving_table",
                    "put down the beef in the left hand on the serving_table",
                    "put down the cheese in the right hand on the serving_table",
                    "turn around to face the ingredients_table",
                    "take a piece of tomato from the ingredients_table with the left hand",
                    "take a piece of bread from the ingredients_table with the right hand",
                    "turn around to face the serving_table",
                    "put down the tomato in the left hand on the serving_table",
                    "put down the bread in the right hand on the serving_table",
                    ],
                    "object_names": ["bread", "lettuce", "beef", "cheese", "tomato", "bread"]},
                "environment_before":env_example(),
                "environment_after":env_example(False),
                "instructioon_summary": "make a cheeseburger",
                "question": ""}

    return json.dumps(out_ex)


def first_prompt():
    # as in 'ChatGPT Empowered Long-Step Robot Control in Various Environments: A Case Application'
    first_prompt = ('You are an excellent interpreter of human instructions for household tasks. ' +
    'Given an instruction and information about the working environment, you break it down into a sequence of robotic actions.' +
    '\nPlease do not start working until I say \"Start working.\" Instead, simply output the message \"Waiting for next input.\"' +
    '\nUnderstood?')

    return first_prompt


def robot_action_list():
    # as in paper
    second_prompt = ('Necessary and sufficient robot actions are defined as follows:\n_________________________\n' +
        '\n\"ROBOT ACTION LIST\"' +
        '\n- take(target_hand: str, target_obj: str): Takes an ingredient from the ingredients_table. '+
        'Takes the input arguments target_hand (str (\'left\' or \'right\'), specifies which hand should pick up the ingredient) '+
        'and target_object (str (\'bread\',\'lettuce\',\'beef\',\'cheese\' or \'tomato\'), specifies which ingredient should be picked up). ' +
        'This command can only be executed one time with each hand before a rotation_180() command has to follow. ' +
        'The target_hand argument cannot be the same in two consecutive take commands. '+
        'For each take command with one hand a put command with that hand has to be performed after rotation.'
        '\n- rotate_180(): Rotates the robot 180 degrees, so that it faces the opposite table. ' +
        'This command has to follow after the take command was performed one or two times. ' +
        'This command has to follow after the put command was performed one or two times. ' +
        '\n- put(target_hand: str): Puts down the ingredient in the selected hand onto the serving_table. '+
        'Takes the input argument target_hand (str (\'left\' or \'right\'), specifies which hand should put down the ingredient). ' +
        'This command can only be executed one time with each hand before a rotation_180() command has to follow. ' +
        'The target_hand argument cannot be the same in two consecutive put commands. '+
        'For each put command with one hand a take command with that hand has to be performed after rotation.'
        '\n\n_________________________\n' +
        '\nThe texts above are part of the overall instruction. Do not start working yet.')

    return second_prompt


def env_and_obj_prompt():
    # as in paper
    third_prompt = ('Information about environments and objects are given as python dictionaries.' +
          'Here is an example of the structure:' +
          '\n\n_________________________\n\n' + json.dumps(env_example()) + '\n\n_________________________\n' +
          '\nThe texts above are part of the overall instruction. Do not start working yet.')

    return third_prompt


def chatGPTs_output():
    # as in paper
    fourth_prompt = ('You divide the actions given in the text into detailed robot actions and put them together as a python dictionary.' +
                    '\nThe dictionary has five keys.' +
                    '\n_________________________' +
                    '\n- dictionary[\"task_cohesion\"]: A dictionary containing information about the robot\'s actions that have been split up.' +
                    '\n- dictionary[\"environment_before\"]: The state of the environment before the manipulation.' +
                    '\n- dictionary[\"environment_after\"]: The state of the environment after the manipulation.' +
                    '\n- dictionary[\"instruction_summary\"]: Contains a brief summary of the given sentence.' +
                    '\n- dictionary[\"question\"]: If you cannot understand the given sentence, you can ask the user to rephrase the sentence. ' +
                    'Leave this key empty if you can understand the given sentence.' +
                    '\n_________________________' +
                    '\nThree keys exist in dictionary[\"task_cohesion\"].' +
                    '\n_________________________' +
                    '\n- dictionary[\"task_cohesion\"][\"task_sequence\"]: Contains a list of robot actions. ' +
                    'Only the behaviors defined in the \"ROBOT ACTION LIST\" will be used.' +
                    '\n- dictionary[\"task_cohesion\"][\"step_instructions\"]: Contains a list of instructions for the robot corresponding to the ' +
                    'list of robot actions.' +
                    '\ndictionary[\"task_cohesion\"][\"object_names\"]: The name/s of the manipulated object/s. Only objects defined in the input' +
                    'dictionary will be used for the object name.' +
                    '\n\n_________________________\n' +
                    '\nThe texts above are part of the overall instruction. Do not start working yet.')

    return fourth_prompt


def in_out_examples():
    # as in paper
    fifth_prompt = ('I will give you an example of the input and the output you will generate.' +
                    '\nExample :' +
                    '\n\n_________________________\n' +
                    '\n- Input:\n' + json.dumps(env_example()) + '\n\nInstruction: Make a cheeseburger.'

                    +'\n_________________________'
                    +'\n- Output:'
                    + output_example()

                    +'\n_________________________'
                    +'\n\n\n'
                    +'\nThe texts above are part of the overall instruction. Do not start working yet.')

    return fifth_prompt


def user_input(env):
    # from paper
    sixth_prompt_1 = ('\nStart working. Resume from the environment below.' +
    '\n_________________________' +
    '\nEnvironment:\n' + json.dumps(env) +
    '\n_________________________'
    '\nInstruction: ')

    sixth_prompt_2 = ('\n_________________________' +
    '\nThe dictionary that you return should be formated as a python dictionary. Follow these rules:' +
    '\n1. Make sure that each element of the [\"step_instructions\"] explains the corresponding element of the [\"task_sequence\"]. ' +
    'Refer to \"ROBOT ACTION LIST\" to understand the elements of [\"task_sequence\"].' +
    '\n2. The length of the [\"step_instructions\"] list must be the same as the length of the [\"task_sequence\"] list.' +
    '\n3. Even though objects disappear, do not delete them from the environment and use the object properties to keep track of all ' +
    'the objects.' +
    '\n4. Make sure that you output a consistent manipulation. For example, put(target_hand=\'left\') should not occur in successive steps. ' +
    'and very importantly take() can only be performed twice before rotate_180() has to be performed. The same goes for put().' +
    '\n5. Never leave \',\' at the end of the list.' +
    '\n6. All keys of the dictionary should be double-quoted.' +
    # these commands are from us (7.-14.)
    '\n7. You have to rotate after put first, before you can take again, and after two take commands you also have to put twice ' +
    'after rotation. Also you cannot grab two objects with one hand, so take with left hand and take with right hand would be a ' +
    'logical sequence, but take with right hand and take with right hand is not. Remember, that the robot has two hands and can ' +
    'therefore perform one take command with each before rotating, but cannot perform two take commmands with one hand before ' +
    'rotating and can also not perform more than one take command with each hand before rotating, so the total number of take ' +
    'commands before rotation cannot exceed two. The same goes for put.'
    '\n8. Remember how a burger is normaly built: Two buns (bread) with the other ingredients between them.'
    '\n9. You cannot perform the take command immediately after the put command, you have to perform a rotate command between ' +
    'take and put. You cannot perform the put command immediately after the take command, you have to perform a rotate command ' +
    'between put and take.'
    '\n10. Your logical sequence must not be (take, rotate, put, take, rotate, put, ...) or (take, put, rotate, take, put, rotate, ...), ' +
    'but it should be (take, rotate, put, rotate, take, rotate, put, rotate, ...) or ' +
    '(take, take, rotate, put, put, rotate, take, take, rotate, put, put, rotate, ...).' +
    '\n11. You do not have to have a logical sequence of the form ' +
    '(take(left), rotate, put(left), rotate, take(right), rotate, put(right), rotate, ...), ' +
    'you can also perform a logical sequence like this instead: ' +
    '(take(left), take(right), rotate, put(left), put(right), rotate, take(left), take(right), rotate, put(left), put(right), rotate, ...).'
    '\n12. Only use the ingredients that were originally on the ingredients table.'
    '\nAdhere to the outputformate I defined above. ' +
    'Think step by step, being aware of what the left/right hand is grabbing or not grabbing.')

    return sixth_prompt_1,sixth_prompt_2


def intitial_prompts(env):
    # function that prompts all of initial prompts
    all_prompts = []
    all_prompts.append(first_prompt())
    all_prompts.append(robot_action_list())
    all_prompts.append(env_and_obj_prompt())
    all_prompts.append(chatGPTs_output())
    all_prompts.append(in_out_examples())
    all_prompts.append(user_input(env)[0])
    all_prompts.append(user_input(env)[1])

    all_instructions = [
        'Copy the prompt and paste it into the ChatGPT prompt-window in your browser. Then come back and press the Next-button.',
        'Copy the prompt and paste it into the ChatGPT prompt-window in your browser. Then come back and press the Next-button.',
        'Copy the prompt and paste it into the ChatGPT prompt-window in your browser. Then come back and press the Next-button.',
        'Copy the prompt and paste it into the ChatGPT prompt-window in your browser. Then come back and press the Next-button.',
        'Copy the prompt and paste it into the ChatGPT prompt-window in your browser. Then come back and press the Next-button.',
        'Insert the Instruction you want to give ChatGPT, then' +
        'copy the prompt and paste it into the ChatGPT prompt-window in your browser. Then come back and press the Next-button.'
    ]

    return all_prompts,all_instructions




