# Testing Instructions

Testing is a critical part of software development. To make it easier to run different test cases on the Nucleo board, we will use the testing framework under the `projects/shared/test_framework`.

First we will walk you through the setup and integration of this framework into existing projects. Then we will discuss how to use this to add your own custom test cases.

## Setup

We assume you have already created your work environment. If not follow the [setup](/setup.md) tutorial first.

To begin we want to create a source folder inside your project that will hold your test cases. If your project already contains the folder 'Tests' with the required files in it, you can skip the steps below.

- Open workspace in CubeIDE
- Under the project explorer window - right click the project - New - source folder
- Name the folder "Tests"
- Click Finish
- Copy files under `projects/template/project/Tests` into your newly created Tests folder
- The file should be `uconfig.h` `utest.c` `utest.h`

You have created your user test and config files and folder. This is what you'll mostly be working with for testing.

- `utest.c` - you will add your different test function here. And adding tests to the test runner (described later)
- `utest.h` - test function declarations
- `uconfig.h` - modify test engine configurations

To run the test in main.c

- Include utest.h in main.c
- Add `#define TEST_MODE` under "private defines"
- In function after right before the while loop add

```C
#ifdef TEST_MODE
  run_tests();
#endif
```

Now after flashing the code and once the main function is invoked, the unit tests should run. If you don't want to run unit test comment out the test mode definition `// #define TEST_MODE`

The only thing left to do now is add the test framework that is stored in the shared folder `projects/shared/test_framework`. This will run through the custom tests you specified in the test runner. Ignore this step if you see the folder "TestEngine" in your project.

- Right click project in the project explorer
- Go to Properties
- Click C/C++ General - Paths and Symbols
- Go to the "Source Location" tab
- Click Link Folder
- Name the folder "TestEngine"
- Check "Link to folder in the file system"
- Paste in `${WORKSPACE_LOC}\com-module-firmware\projects\shared\test_framework\`
- Click "Ok"
- Go to the includes tab
- Click "Add" under the includes section
- Select workspace folder and click the "TestEngine" folder you just created
- Click "Ok"
- Click "Apply and Close"

Try a build and see if it compiles without any errors. You should be done with the setup otherwise.

## Custom Test Creation

Now that you are have setup the testing framework, we will walk through creating and running new test case.

For this example, let's say we want to create a test to rotate the motor and print out it's position.

### Step 1 - Write a test function

The first step would be to create a test function.

Let's call this function `run_motor_test()`

Add the function definition inside `utest.h`. The function should follow the format `testresult nameofFunc(void)`

```C
testresult run_motor_test(void);
```

Then add the implementation of the function in `utest.c`.

```C
testresult run_motor_test(void) {
	testresult res = {TSUCCESS, {0}};
    move_motor_now();

	for (int i = 0 ; i < MOVE_SECS; ++i) {
        const int pos;

		res.error.raw = read_motor_pos(&pos);
		if (res.error.raw != OK) {
			res.stat = TERROR;
			return res;
		}

        printf("%d", pos);

		HAL_Delay(1000);
	}
	return res;
}
```

You can add the MOVE_SECS defintion at the top of the file. Any test case specific defintions or variables can be added there.

`testresult` is a struct that contains the status of your test (TSUCCESS or TERROR) and the error value (0 by default).

### Step 2 - Add test to test runner

Now that you have a test implemented. We can simply add it to the test runner at the top of the `utest.c` file.

```C
// -- Add to test runner here --
const t_test test_runner[] = {
//		{"Name of test", "function definition", "testgroup id"
		{.testname="Running motor test", .func=run_motor_test, .group=MOTORS}
};
```

Once it is in the test_runner, the test engine in the testing framework should take care of the rest (scheduling, running and printing test results).

By default all tests in your test runner will run in sequence. But this is not always desired.

Thats where the `group` attribute comes in. It is used to add a tag to your test so that you can filter for specific test cases (for the times when you only hardware setup for a subset of these test).

Groups configurations are shared with the engine. So it is inside the `uconfig.h` file.

You can a test group here inside the testgroup enum. For this example let's add the MOTORS group.

```C
typedef enum {
	ALL,
	MOTORS
} testgroup;
```

To select which test group to run, edit the TEST_GROUP_SEL. If you want all the tests in the runner to run, define it with `ALL`.

```C
#define TEST_GROUP_SEL MOTORS
```

There you go! you've created your first test. That's all there is to it. You can now flash to code into your board and the unit tests in the test runner should run at the start.

You can view the test output through UART on Putty.

Note: If you don't want unit test to run at the start, make sure to comment out `#define TEST_MODE` in `main.c`
