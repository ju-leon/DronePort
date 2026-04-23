#include <chrono>
#include <atomic>
#include <thread>
#include <functional>
#include <iostream>

using namespace std::chrono_literals;

void StartThread(
    std::thread& thread,
    std::atomic<bool>& running,
    const std::function<bool(void)> &process, // renamed from "Process": parameters are values, snake_case fits the rest of the signature
    const std::chrono::seconds timeout)
{
    thread = std::thread(
        // change from [&]
        // to [&running, process, timeout]
        // to capture the variables by value, so that they are copied to the thread's context
        // and remain valid even after StartThread returns
        [&running, process, timeout] ()
        {
            auto start = std::chrono::high_resolution_clock::now();
            while(running)
            {
                bool aborted = process();

                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                if (aborted || duration > timeout)
                {
                    running = false;
                    break;
                }
            }
        });
}

int main()
{
    // direct-initialize: std::atomic<bool> is non-copyable, so "= true" would
    // invoke the deleted copy constructor under strict compilers
    std::atomic<bool> my_running{true};
    std::thread my_thread1, my_thread2;
    int loop_counter1 = 0, loop_counter2 = 0;

    // start actions in separate threads and wait for them
    StartThread(
        my_thread1,
        my_running,
        [&]()
        {
            // "some actions" simulated with waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            loop_counter1++;
            return false;
        },
        10s); // loop timeout

    StartThread(
        my_thread2,
        my_running,
        [&]()
        {
            // "some actions" simulated with waiting
            if (loop_counter2 < 5)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                loop_counter2++;
                return false;
            }
            return true;
        },
        10s); // loop timeout


    my_thread1.join();
    my_thread2.join();

    // print execution loop counters
    std::cout << "C1: " << loop_counter1 << " C2: " << loop_counter2 << std::endl;
}


// What does this code do?
// 1. Starts two threads that execute some actions in a loop until either of them signals to stop or a timeout occurs.
// 2. Each thread runs a loop that simulates continuously repeating some actions.
// 3. Both threads count how many times they have executed their loop and print the counts at the end.
// 4. If either thread signals to stop (by returning true from the process function) or if the loop runs into a timeout,
//    the threads will stop executing and the main thread will print the loop counters.

// Problems with this code:
// 1. If either thread signals to stop (by returning true from the process function),
//    the other thread will also stop executing, even if it has not reached its timeout.
//    This may be intended, but from the signature of "StartThread", I would expect that a
//    timeout in one thread should not stop the other.
//    To fix this, the "if (aborted || duration > timeout)" condition inside the thread's loop
//    could be changed to
//    "if (aborted) { running = false; break; } else if (duration > timeout) { break; }"
//    This way, a timeout only stops the thread that hit it, while an explicit abort still
//    sets "running = false" and signals all threads to stop.
//
// 2. The lambda functions inside StartThread captured everything by reference,
//    which can lead to issues once the captured variables go out of scope.
//    By copying the function "process" to the thread, we ensure that the thread owns its own copy of the function
//    (and timeout), preventing potential dangling references (in theory, same problem could occur with "running",
//    but since this variable is used to signal to the thread to stop,
//    it has to be passed by reference, so a user can expect that this should not go out of scope until all threads stop).
//
//    Although in this specific code, the captured variables are still valid because they are defined in main
//    and will not go out of scope until main returns, but for an unsuspecting user of StartThread,
//    this could lead to bugs if they capture variables that do go out of scope.