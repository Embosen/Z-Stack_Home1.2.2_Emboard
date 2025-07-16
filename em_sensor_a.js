const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const ea = require('zigbee-herdsman-converters/lib/exposes').access;
const e = exposes.presets;

// 设备验证函数
const isEmboardDevice = (device) => {
    return device && 
           device.modelID === 'Em-Sensor-A' && 
           device.manufacturerName === 'Emboard.Co';
};

module.exports = [
    {
        fingerprint: [
            {modelID: 'Em-Sensor-A', manufacturerName: 'Emboard.Co'},
        ],
        model: 'Em-Sensor-A',
        vendor: 'Emboard',
        description: 'Emboard Zigbee Sensor',
        fromZigbee: [
            {
                cluster: 'genOnOff',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    // 验证设备身份
                    if (!isEmboardDevice(msg.device)) {
                        return;
                    }
                    
                    if (msg.data.hasOwnProperty('onOff')) {
                        const state = msg.data['onOff'] === 1; // true or false
                        const endpoint = msg.endpoint.ID;

                        console.log(`OnOff report - endpoint ${endpoint}: ${state}`);

                        if (endpoint === 10) {
                            publish({button1: state});
                        } else if (endpoint === 11) {
                            publish({button2: state});
                        } else if (endpoint === 8) {
                            publish({state_green: state ? 'ON' : 'OFF'});
                        } else if (endpoint === 9) {
                            publish({state_red: state ? 'ON' : 'OFF'});
                        }
                    }
                },
            },
        ],
        toZigbee: [tz.on_off],
        exposes: [
            e.switch().withEndpoint('green').withLabel('Green LED').withDescription('Green LED switch'),   // 绿色LED开关
            e.switch().withEndpoint('red').withLabel('Red LED').withDescription('Red LED switch'),     // 红色LED开关
            e.binary('button1', ea.STATE, true, false).withLabel('Button 1').withDescription('Button 1 sensor'), // P1_2按键传感器
            e.binary('button2', ea.STATE, true, false).withLabel('Button 2').withDescription('Button 2 sensor'), // P1_3按键传感器
        ],
        icon: 'device_icons/em_sensor_a.png',
        endpoint: (device) => {
            // 验证设备身份
            if (!isEmboardDevice(device)) {
                return {};
            }
            
            return {
                green: 8,    // 绿色LED端点
                red: 9,       // 红色LED端点
                button1: 10,  // P1_2按键传感器端点
                button2: 11,  // P1_3按键传感器端点
            };
        },
        configure: async (device, coordinatorEndpoint, logger) => {
            // 验证设备身份
            if (!isEmboardDevice(device)) {
                console.log(`Skipping configuration for non-Emboard device: ${device.modelID} from ${device.manufacturerName}`);
                return;
            }
            
            try {
                // 绑定所有端点
                const endpoints = [8, 9, 10, 11];
                for (const endpointId of endpoints) {
                    const endpoint = device.getEndpoint(endpointId);
                    if (endpoint) {
                        await endpoint.bind('genOnOff', coordinatorEndpoint);
                        console.log(`Bound genOnOff cluster on endpoint ${endpointId}`);
                    }
                }
            } catch (error) {
                console.error(`Error binding endpoints: ${error}`);
            }
        },
    },
]; 
