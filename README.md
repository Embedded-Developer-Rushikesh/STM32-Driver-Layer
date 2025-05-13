<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>GPIO Driver Layer - ARM Cortex-Mx (STM32F4x)</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      line-height: 1.6;
      background-color: #f4f4f4;
      color: #333;
      margin: 0;
      padding: 20px;
    }
    h1, h2, h3 {
      color: #00529B;
    }
    .section {
      background: #fff;
      padding: 20px;
      margin-bottom: 20px;
      border-radius: 8px;
      box-shadow: 0 0 10px rgba(0,0,0,0.1);
    }
    code {
      background: #eee;
      padding: 2px 4px;
      border-radius: 4px;
      font-family: Consolas, monospace;
    }
  </style>
</head>
<body>

  <h1>GPIO Driver Layer for ARM Cortex-Mx (STM32F4x)</h1>

  <div class="section">
    <h2>1. Peripheral Base Addresses</h2>
    <ul>
      <li><strong>PERIPH_BASE:</strong> <code>0x40000000</code></li>
      <li><strong>APB1PERIPH_BASE:</strong> <code>0x40000000</code></li>
      <li><strong>APB2PERIPH_BASE:</strong> <code>0x40010000</code></li>
      <li><strong>AHB1PERIPH_BASE:</strong> <code>0x40020000</code></li>
      <li><strong>AHB2PERIPH_BASE:</strong> <code>0x50000000</code></li>
    </ul>
  </div>

  <div class="section">
    <h2>2. GPIO Base Addresses (AHB1 Bus)</h2>
    <ul>
      <li><strong>GPIOA:</strong> <code>0x40020000</code></li>
      <li><strong>GPIOB:</strong> <code>0x40020400</code></li>
      <li><strong>GPIOC:</strong> <code>0x40020800</code></li>
      <!-- Add more as needed -->
    </ul>
  </div>

  <div class="section">
    <h2>3. GPIO Register Offsets</h2>
    <ul>
      <li><code>MODER</code> (Mode Register): <code>0x00</code></li>
      <li><code>OTYPER</code> (Output Type Register): <code>0x04</code></li>
      <li><code>OSPEEDR</code> (Output Speed Register): <code>0x08</code></li>
      <li><code>PUPDR</code> (Pull-Up/Pull-Down Register): <code>0x0C</code></li>
      <li><code>IDR</code> (Input Data Register): <code>0x10</code></li>
      <li><code>ODR</code> (Output Data Register): <code>0x14</code></li>
      <li><code>BSRR</code> (Bit Set/Reset Register): <code>0x18</code></li>
    </ul>
  </div>

  <div class="section">
    <h2>4. SPI1 Register Addresses (APB2)</h2>
    <p>SPI1 Base: <code>0x40013000</code></p>
    <ul>
      <li>Control Register 1: <code>0x40013000</code></li>
      <li>Control Register 2: <code>0x40013004</code></li>
      <li>Status Register: <code>0x40013008</code></li>
      <li>Data Register: <code>0x4001300C</code></li>
      <li>CRC Polynomial Register: <code>0x40013010</code></li>
      <li>RX CRC Register: <code>0x40013014</code></li>
      <li>TX CRC Register: <code>0x40013018</code></li>
      <li>I2S Config Register: <code>0x4001301C</code></li>
      <li>I2S Prescaler: <code>0x40013020</code></li>
    </ul>
  </div>

  <div class="section">
    <h2>5. GPIO Configuration Structure (C-Style)</h2>
    <pre><code>typedef struct {
    uint32_t MODER;
    uint32_t OTYPER;
    uint32_t OSPEEDR;
    uint32_t PUPDR;
    uint32_t IDR;
    uint32_t ODR;
    uint32_t BSRR;
} GPIO_RegDef_t;</code></pre>
  </div>

  <div class="section">
    <h2>6. GPIO Driver API Requirements</h2>
    <ul>
      <li>Initialize GPIO</li>
      <li>Enable/Disable GPIO Clock</li>
      <li>Read from GPIO Pin/Port</li>
      <li>Write to GPIO Pin/Port</li>
      <li>Configure Alternate Function</li>
      <li>Configure Interrupt</li>
      <li>Interrupt Handling</li>
    </ul>
  </div>

  <div class="section">
    <h2>7. Interrupt Configuration for GPIO</h2>
    <ol>
      <li>Configure pin as input</li>
      <li>Configure edge detection in EXTI (RT/FT/Both)</li>
      <li>Enable interrupt in EXTI IMR</li>
      <li>Map GPIO to EXTI line via SYSCFG_EXTICR</li>
      <li>Set IRQ priority in NVIC_IPRx</li>
      <li>Enable IRQ in NVIC_ISERx</li>
      <li>Implement IRQ Handler</li>
    </ol>
  </div>

  <div class="section">
    <h2>8. NVIC Configuration Registers</h2>
    <ul>
      <li><strong>NVIC_ISER0 - ISER2:</strong> Set-enable IRQs (Enable = 1)</li>
      <li><strong>NVIC_ICER0 - ICER2:</strong> Clear-enable IRQs (Disable = 1)</li>
      <li><strong>NVIC_IPR0 - IPR59:</strong> Set priority</li>
    </ul>
  </div>

</body>
</html>
