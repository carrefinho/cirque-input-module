This is a fork of [petejohanson/cirque-input-module](https://github.com/petejohanson/cirque-input-module) with added cursor inertia/trackball simulation functionality.

## Usage

Configure the Cirque trackpad in devicetree like so:

```devicetree
&spi2 {
   status = "okay";
   cs-gpios = <&xiao_d 0 GPIO_ACTIVE_LOW>;

   glidepoint: glidepoint@0 {
      compatible = "cirque,pinnacle";
      reg = <0>;
      spi-max-frequency = <1000000>;
      status = "okay";
      dr-gpios = <&xiao_d 7 (GPIO_ACTIVE_HIGH)>;

      sensitivity = "2x";
      sleep;
      no-taps;

      z-touch-detection;
      z-threshold-touch = <5>;
      z-threshold-release = <2>;
      inertia;
      inertia-start-velocity = <300>;
      inertia-stop-velocity = <20>;
   };
};
```