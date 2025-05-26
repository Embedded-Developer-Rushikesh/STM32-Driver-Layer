<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>STM32F446xx GPIO Driver Documentation</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            line-height: 1.6;
            color: #333;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f9f9f9;
        }
        
        header {
            background-color: #005f87;
            color: white;
            padding: 20px;
            border-radius: 5px;
            margin-bottom: 30px;
        }
        
        h1, h2, h3 {
            color: #005f87;
        }
        
        h2 {
            border-bottom: 2px solid #005f87;
            padding-bottom: 5px;
            margin-top: 30px;
        }
        
        code {
            background-color: #f0f0f0;
            padding: 2px 5px;
            border-radius: 3px;
            font-family: 'Courier New', Courier, monospace;
        }
        
        pre {
            background-color: #f5f5f5;
            padding: 15px;
            border-left: 4px solid #005f87;
            overflow-x: auto;
            border-radius: 3px;
        }
        
        .api-table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }
        
        .api-table th, .api-table td {
            border: 1px solid #ddd;
            padding: 12px;
            text-align: left;
        }
        
        .api-table th {
            background-color: #005f87;
            color: white;
        }
        
        .api-table tr:nth-child(even) {
            background-color: #f2f2f2;
        }
        
        .example {
            background-color: #e6f7ff;
            padding: 15px;
            border-left: 4px solid #0099ff;
            margin: 20px 0;
            border-radius: 3px;
        }
        
        .note {
            background-color: #fff8e6;
            padding: 15px;
            border-left: 4px solid #ffcc00;
            margin: 20px 0;
            border-radius: 3px;
        }
    </style>
</head>
<body>
    <header>
        <h1>STM32F446xx GPIO Driver</h1>
        <p>Comprehensive General Purpose Input/Output driver for STM32F446xx microcontrollers</p>
    </header>

    <section id="overview">
        <h2>Overview</h2>
        <p>This driver provides complete control over the GPIO peripherals in STM32F446xx microcontrollers. It supports all GPIO features including input/output configuration, alternate functions, interrupt handling, and various pin settings.</p>
    </section>

    <section id="features">
        <h2>Features</h2>
        <ul>
            <li>Support for all 16 GPIO pins (0-15) per port</li>
            <li>Multiple operation modes:
                <ul>
                    <li>Input/output modes</li>
                    <li>Alternate function configuration</li>
                    <li>Analog mode</li>
                    <li>Interrupt modes (rising/falling/both edges)</li>
                </ul>
            </li>
            <li>Push-pull and open-drain output configurations</li>
            <li>Configurable output speed (low, medium, fast, high)</li>
            <li>Pull-up/pull-down resistor control</li>
            <li>Comprehensive interrupt handling capabilities</li>
        </ul>
    </section>

    <section id="api-reference">
        <h2>API Reference</h2>
        
        <h3>Initialization/Configuration</h3>
        <table class="api-table">
            <tr>
                <th>Function</th>
                <th>Description</th>
                <th>Parameters</th>
            </tr>
            <tr>
                <td><code>GPIO_PeriClockControl()</code></td>
                <td>Controls peripheral clock for GPIO ports</td>
                <td>
                    <code>GPIO_RegDef_t *pGPIOx</code>: GPIO port<br>
                    <code>uint8_t EnorDi</code>: Enable/disable
                </td>
            </tr>
            <tr>
                <td><code>GPIO_Init()</code></td>
                <td>Initializes GPIO pins with specified configuration</td>
                <td><code>GPIO_Handle_t *pGPIOHandle</code>: GPIO handle</td>
            </tr>
            <tr>
                <td><code>GPIO_DeInit()</code></td>
                <td>Resets GPIO port configuration</td>
                <td><code>GPIO_RegDef_t *pGPIOx</code>: GPIO port</td>
            </tr>
        </table>

        <h3>Data Operations</h3>
        <table class="api-table">
            <tr>
                <th>Function</th>
                <th>Description</th>
                <th>Parameters</th>
            </tr>
            <tr>
                <td><code>GPIO_ReadFromInputPin()</code></td>
                <td>Reads state of a single input pin</td>
                <td>
                    <code>GPIO_RegDef_t *pGPIOx</code>: GPIO port<br>
                    <code>uint8_t PinNumber</code>: Pin number (0-15)
                </td>
            </tr>
            <tr>
                <td><code>GPIO_WriteToOutputPin()</code></td>
                <td>Writes to a single output pin</td>
                <td>
                    <code>GPIO_RegDef_t *pGPIOx</code>: GPIO port<br>
                    <code>uint8_t PinNumber</code>: Pin number (0-15)<br>
                    <code>uint8_t Value</code>: Value to write (0 or 1)
                </td>
            </tr>
            <tr>
                <td><code>GPIO_ToggleOutputPin()</code></td>
                <td>Toggles the state of an output pin</td>
                <td>
                    <code>GPIO_RegDef_t *pGPIOx</code>: GPIO port<br>
                    <code>uint8_t PinNumber</code>: Pin number (0-15)
                </td>
            </tr>
        </table>

        <h3>Interrupt Handling</h3>
        <table class="api-table">
            <tr>
                <th>Function</th>
                <th>Description</th>
                <th>Parameters</th>
            </tr>
            <tr>
                <td><code>GPIO_IRQInterruptConfig()</code></td>
                <td>Enables/disables GPIO interrupts</td>
                <td>
                    <code>uint8_t IRQNumber</code>: Interrupt number<br>
                    <code>uint8_t EnorDi</code>: Enable/disable
                </td>
            </tr>
            <tr>
                <td><code>GPIO_IRQPriorityConfig()</code></td>
                <td>Sets interrupt priorities</td>
                <td>
                    <code>uint8_t IRQNumber</code>: Interrupt number<br>
                    <code>uint32_t IRQPriority</code>: Priority level
                </td>
            </tr>
            <tr>
                <td><code>GPIO_IRQHandling()</code></td>
                <td>Handles GPIO interrupt events</td>
                <td><code>uint8_t PinNumber</code>: Pin number (0-15)</td>
            </tr>
        </table>
    </section>

    <section id="configuration">
        <h2>Configuration Macros</h2>
        
        <h3>Pin Modes</h3>
        <pre>
#define GPIO_MODE_IN        0   // Input mode
#define GPIO_MODE_OUT       1   // Output mode
#define GPIO_MODE_ALTFN     2   // Alternate function
#define GPIO_MODE_ANALOG    3   // Analog mode
#define GPIO_MODE_IT_FT     4   // Falling edge interrupt
#define GPIO_MODE_IT_RT     5   // Rising edge interrupt
#define GPIO_MODE_IT_RFT    6   // Both edges interrupt</pre>

        <h3>Output Types</h3>
        <pre>
#define GPIO_OP_TYPE_PP     0   // Push-pull
#define GPIO_OP_TYPE_OD     1   // Open-drain</pre>

        <h3>Speed Settings</h3>
        <pre>
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPOI_SPEED_HIGH     3</pre>
    </section>

    <section id="examples">
        <h2>Usage Examples</h2>
        
        <div class="example">
            <h3>LED Blinking Example</h3>
            <pre>
// Configure LED on PA5
GPIO_Handle_t GpioLed;

GpioLed.pGPIOx = GPIOA;
GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

// Initialize GPIO
GPIO_PeriClockControl(GPIOA, ENABLE);
GPIO_Init(&GpioLed);

// Blink LED
while(1) {
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
    Delay(500);
}</pre>
        </div>

        <div class="example">
            <h3>Button Interrupt Example</h3>
            <pre>
// Configure button on PC13 with interrupt
GPIO_Handle_t GpioButton;

GpioButton.pGPIOx = GPIOC;
GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

// Initialize GPIO
GPIO_PeriClockControl(GPIOC, ENABLE);
GPIO_Init(&GpioButton);

// Configure interrupt
GPIO_IRQInterruptConfig(EXTI15_10_IRQn, ENABLE);
GPIO_IRQPriorityConfig(EXTI15_10_IRQn, 15);

// Interrupt handler
void EXTI15_10_IRQHandler(void) {
    GPIO_IRQHandling(GPIO_PIN_NO_13);
    // Handle button press
}</pre>
        </div>
    </section>

    <section id="dependencies">
        <h2>Dependencies</h2>
        <ul>
            <li>STM32F446xx device header files</li>
            <li>CMSIS core headers</li>
            <li>Standard C library</li>
        </ul>
    </section>
</body>
</html>
