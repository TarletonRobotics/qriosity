const Board = require('firmata')

function main() {
    console.log("starting firmata")

    Board.requestPort((error, port) => {
          if (error) {
            console.log(error);
            return;
          }

          const board = new Board(port.comName);

          board.on("ready", () => {
            console.log("READY");

            const HW_SERIAL1 = board.SERIAL_PORT_IDs.HW_SERIAL1;

            const SW_SERIAL0 = board.SERIAL_PORT_IDs.SW_SERIAL0;

            board.serialConfig({
              portId: HW_SERIAL1,
              baud: 9600
            });

            board.serialRead(HW_SERIAL1, data => {
              console.log(new Buffer(data).toString("ascii"));
            });

            board.on("string", message => {
              console.log(message);
            });

            // log serial pin numbers
            for (const pin in board.pins) {
              const modes = board.pins[pin].supportedModes;
              for (const mode in modes) {
                if (modes[mode] === board.MODES.SERIAL) {
                  console.log(`serial pin: ${pin}`);
                }
              }
            }
            // 
            console.log('sending string: How are you' );
            var bytes = new Buffer("How are you.", 'utf8');
            var data = [];
            
            data.push(board.STRING_DATA);
            board.sendString("Hello\0");
            data.push(board.END_SYSEX);
            
          });
        });


}

main();