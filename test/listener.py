import requests
import time
import json
import multiprocessing
import threading
import sys

ROBOT_ID = 1  # The ID assigned to this robot in the database
BASE_URL = 'http://192.168.28.99:8000'
SERVER_URL = f'{BASE_URL}/robot_code/{ROBOT_ID}'
EXEC_STATUS_URL = f'{BASE_URL}/robot_code/{ROBOT_ID}/exec'
FINISH_PROGRAM_URL = f'{BASE_URL}/robot_code/{ROBOT_ID}/finish_program/'
filename = 'student_code11.py'
headers = {'Content-Type': 'application/json'}

def execute_code(code, output_queue):
    try:
        # Redirect stdout to the queue
        class WriteToQueue(object):
            def __init__(self, queue):
                self.queue = queue
            def write(self, msg):
                self.queue.put(msg)
            def flush(self):
                pass
        sys.stdout = WriteToQueue(output_queue)
        import Arm_Lib_New
        # Execute the student's code
        exec_globals = {'__builtins__':__builtins__,'Arm_Lib_New':Arm_Lib_New}
        exec_locals = {}
        exec(code, exec_globals)
    except Exception as e:
        output_queue.put(f"Error executing code: {e}\n")
    finally:
        sys.stdout = sys.__stdout__

def output_reader(q):
    while True:
        msg = q.get()
        if msg == 'STOP':
            break
        print(msg, end='')

if __name__ == '__main__':
    while True:
        response = requests.get(SERVER_URL)

        if response.status_code == 200:
            data = response.json()
            code = data.get('code')
            program_id = data.get('program_id')
            print(program_id)

            if code is None or program_id is None:
                print('Invalid response data')
                time.sleep(15)
                continue

            # Save the code
            with open(filename, 'w') as file:
                file.write(code)
                print(f"Code received:\n{code}")

            stop = False  # Reset stop flag for new code
            while not stop:
                execResponse = requests.get(EXEC_STATUS_URL)

                if execResponse.status_code == 200:
                    status = execResponse.text.strip()
                    print(f"Execution status: {status}")

                    if status == 'Run':
                        # Start executing the code in a separate process
                        print('Attempting to run the code')

                        # Create a queue to get output from the child process
                        output_queue = multiprocessing.Queue()
                        process = multiprocessing.Process(target=execute_code, args=(code, output_queue))
                        process.start()

                        # Start a thread to read from the output_queue and print to console
                        output_thread = threading.Thread(target=output_reader, args=(output_queue,))
                        output_thread.start()

                        # Monitor the execution status
                        while True:
                            time.sleep(2)  # Check every 2 seconds
                            execResponse = requests.get(EXEC_STATUS_URL)
                            if execResponse.status_code == 200:
                                status = execResponse.text.strip()
                                if status == 'Stop':
                                    print("Stop command received. Terminating execution.")
                                    process.terminate()
                                    process.join()
                                    output_queue.put('STOP')  # Signal the output thread to stop
                                    output_thread.join()
                                    stop = True
                                    break
                                elif not process.is_alive():
                                    # Execution finished
                                    print("Code execution finished.")
                                    output_queue.put('STOP')  # Signal the output thread to stop
                                    output_thread.join()
                                    stop = True
                                    break
                            else:
                                print(f"Error fetching exec status: {execResponse.status_code}")
                                continue

                        # After execution or termination, notify server
                        data = {'program_id': program_id}
                        print(f"Sending data to finish_program: {data}")
                        Finishresponse = requests.post(FINISH_PROGRAM_URL, data=json.dumps(data), headers=headers)

                        if Finishresponse.status_code == 200:
                            print('Program status updated to Finished')
                        else:
                            print('Failed to update program status:', Finishresponse.text)
                        break  # Exit the inner while loop

                    elif status == 'Waiting':
                        print('Waiting for student to start program')
                    elif status == 'Stop':
                        print('Program execution has been stopped')
                        stop = True
                    else:
                        print('Error Condition')
                else:
                    print(f"Error fetching exec status: {execResponse.status_code}")
                time.sleep(3)

        else:
            print(f"Error fetching code: {response.status_code}")
        time.sleep(5)  # Wait before the next request

