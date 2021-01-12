/**
 * Arduino Style Guide
 * @author  Karn Watcharasupat
 * @version Dec 2020
 */

#include <library.h>

#include "localLibrary.h"

// use all caps for global constant
// this tells everyone that this is not a mutable variable
#define GLOBAL_CONST_USE_ALL_CAPS_UNDERSCORE 0

// `DEBUG` tag is a helpful tag to help you wrap debug code without having to delete it
#define DEBUG 0    
// `VERBOSE` tag let you suppresses printing without incurring extra computation
#define VERBOSE 0  

// try to group relevant globals together
#define USE_SERIAL 1
#define BAUD_RATE 9600

#pragma region ExampleDebugVerbose

/**
 * @brief  This function demonstrates how to use conditional directives
 * @param  value: input number
 * @param  verify: validation number
 * @param  successString: string to print if `value` and `verify` are equal, ignored if `VERBOSE` is off
 * @param  failureString: string to print if `value` and `verify` are not equal, ignored if `VERBOSE` is off
 * @retval true if `value` and `verify` are equal, false otherwise
 */
bool exampleDebugVerbose(int value, int verify, char* successString, char* failureString) {

    // you can use conditional directive as if you are using a normal if block
    #if (DEBUG)
    // this block only gets compiled if `DEBUG` is true
        bool correct = (value == verify);
    #else
    // this block only gets compiled if `DEBUG` is false
        bool correct = true;
    #endif

    #if (VERBOSE && USE_SERIAL)
        // this block only gets compiled if `VERBOSE` is true
        if (correct) {
            Serial.println(successString);

        } else {
            Serial.println(failureString);
        }
    #endif
    
    return correct;
}

#pragma endregion

// use `#pragma region regionName` to define collapsible sections in your code and group relevant functions together
#pragma region RegionName

/**
 * @brief  Describe what your function does
 * @note   Additional notes if needed
 * @param  variableNameUseCamelCase:    specifics about your argument
 * @param  arg2:                        specifics about your argument
 * @retval specifics about your return value
 */
int normalFunctionNameUseCamelCase(int variableNameUseCamelCase, float arg2) {
    return 0;
}

/**
 * @brief  Describe what your function does
 * @note   Additional notes if needed
 * @param  arg1:    specifics about your argument
 * @param  arg2:                        specifics about your argument
 * @retval specifics about your return value
 */
int _privateFunctionNameAddUnderscoreInFront(int arg1, float arg2) {
    // private functions are usually helper function that should not be called in the `main` section of your code
    // adding underscore in front usually helps to denote this
    return 0;
}
#pragma endregion

#pragma region Main

// there is no need to add documentation for `setup()` and `void()`

void setup() {
    #if (USE_SERIAL)
        Serial.begin(BAUD_RATE);
    #endif

    initThis();
    initThat();
    initAThousandMoreThings();
    tryNotToHaveLooseCodesInHere();
    itIsVeryHardToDebug();
    ifThereAreTooManyThingsFloatingAround();
}

void loop() {
    // do something and repeat forever
}
#pragma endregion