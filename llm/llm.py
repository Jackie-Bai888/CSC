from http import HTTPStatus
import dashscope
dashscope.api_key = 'sk-6b0fb87804c14407be5c226bcaa7d7b6'
from dashscope import Generation
from dashscope.api_entities.dashscope_response import Role


def extract_information(danger_context):
    messages = [{'role': Role.SYSTEM, 'content': 'You are a important information extractor. Your task is to extract the subject of dangerous actions, dangerous behaviors, the position of the subject of dangerous actions relative to the ego vehicle, and the weather from the dangerous driving text I provided.  '
                                                 'You need to give the four information topics and content in dictionary format. If the information of a topic does not exist, the content of the topic needs to be set to empty. '
                                                 'In addition, the value of dictionary needs to be presented in the form of words or phrases. For example, The dangerous driving text is that on a rainy day, the car in front of the left suddenly braked. The extracted information is that'
                                                 '{'
                                                 ' "subject of dangerous action": "vehicle",'
                                                 ' "dangerous behavior": "sudden brake",'
                                                 ' "position of subject relative to ego vehicle": "front left",'
                                                 '"weather" : "rain"}'},
                {'role': Role.USER, 'content': danger_context}]
    # 通义千问2.1是基于qwen-max大模型
    response = Generation.call(
        Generation.Models.qwen_turbo,
        messages = messages,
        result_format='message',  # set the result to be "message" format.
    )
    if response.status_code == HTTPStatus.OK:
        # print(response)
        # append result to messages.
        extract_info = response.output.choices[0]['message']['content']
        return extract_info
    else:
        print('Request id: %s, Status code: %s, error code: %s, error message: %s' % (
            response.request_id, response.status_code,
            response.code, response.message
        ))
        return 'error'

def find_matching_behavior(phrase_subject, sub_list, phrase_behavior, beh_list, phrase_position, pos_list, phrase_weather, weather_list):
    ori_data = [phrase_subject, sub_list, phrase_behavior, beh_list, phrase_position, pos_list, phrase_weather, weather_list]
    messages = [{'role': Role.SYSTEM, 'content': 'You are a synonym recognizer. Your task is to find the closest word from the word list when I give you a word or phrase and a word list. The format of the output content is a dictionary and the key of the dictionary is output. '
                                                 'If the word or phrase I give you is empty, then the value of the dictionary is also empty. For example, '
                                                 'The word or phrase is that ahead.'
                                                 'The word list that ["left front","front","right front","left side","right side","left rear","behind","right rear"].'
                                                 'The closest word is that '
                                                 '{"output": "front"}'},
                {'role': Role.USER, 'content': 'The phrase is %s. The word list is %s' % (ori_data[0], ori_data[1])}]
    match_result = []
    # 通义千问2.1是基于qwen-max大模型
    for i in range(1, 5):
        response = Generation.call(
            Generation.Models.qwen_turbo,
            messages = messages,
            result_format = 'message',  # set the result to be "message" format.
        )
        if response.status_code == HTTPStatus.OK:
            # print(response)
            # append result to messages.
            result = response.output.choices[0]['message']['content']
            result = eval(result)["output"]
            match_result.append(result)
            messages.append({'role': response.output.choices[0]['message']['role'],
                             'content': response.output.choices[0]['message']['content']})
        else:
            print('Request id: %s, Status code: %s, error code: %s, error message: %s' % (
                response.request_id, response.status_code,
                response.code, response.message
            ))
        if i < 4:
            messages.append({'role': Role.USER, 'content': 'The phrase is %s. The word list is %s' % (ori_data[i*2], ori_data[i*2+1])})
    return match_result

if __name__ == '__main__':
    extract_information()
