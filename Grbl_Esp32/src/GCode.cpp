/*
  GCode.cpp - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

	2018 -	Bart Dring This file was modifed for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Grbl.h"
#include "Machines\Cool_Draw.h"
#include "Stepper.h"

// Allow iteration over CoordIndex values
CoordIndex& operator++(CoordIndex& i) {
    i = static_cast<CoordIndex>(static_cast<uint8_t>(i) + 1);
    return i;
}

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.
static const int32_t MaxLineNumber = 10000000;
static const uint8_t MaxToolNumber = 255;  // Limited by max unsigned 8-bit value

// Declare gc extern struct
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return (status);

void gc_init() {
    // Reset parser state:
    memset(&gc_state, 0, sizeof(parser_state_t));
    // Load default G54 coordinate system.
    gc_state.modal.coord_select = CoordIndex::G54;
    coords[gc_state.modal.coord_select]->get(gc_state.coord_system);
}

// 设置g代码解析器的位置，单位为mm。由系统调用abort和hardSets g代码解析器的位置以毫米为单位。按步骤输入。由系统调用abort和hard
//限制拉出动作。 limit pull-off routines.
//相当于gc_state.position=sys_position/步数
void gc_sync_position() {
    system_convert_array_steps_to_mpos(gc_state.position, sys_position);
}

// Edit GCode line in-place, removing whitespace and comments and
// converting to uppercase
void collapseGCode(char* line) {
    // parenPtr, if non-NULL, is the address of the character after (
    char* parenPtr = NULL;
    // outPtr is the address where newly-processed characters will be placed.
    // outPtr is alway less than or equal to inPtr.
    char* outPtr = line;
    char  c;
    for (char* inPtr = line; (c = *inPtr) != '\0'; inPtr++) {
        if (isspace(c)) {
            continue;
        }
        switch (c) {
            case ')':
                if (parenPtr) {
                    // Terminate comment by replacing ) with NUL
                    *inPtr = '\0';
                    report_gcode_comment(parenPtr);
                    parenPtr = NULL;
                }
                // Strip out ) that does not follow a (
                break;
            case '(':
                // Start the comment at the character after (
                parenPtr = inPtr + 1;
                break;
            case ';':
                // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
#ifdef REPORT_SEMICOLON_COMMENTS
                report_gcode_comment(inPtr + 1);
#endif
                *outPtr = '\0';
                return;
            case '%':
                // TODO: Install '%' feature
                // Program start-end percent sign NOT SUPPORTED.
                // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
                // where, during a program, the system auto-cycle start will continue to execute
                // everything until the next '%' sign. This will help fix resuming issues with certain
                // functions that empty the planner buffer to execute its task on-time.
                break;
            case '\r':
                // In case one sneaks in
                break;
            default:
                if (!parenPtr) {
                    *outPtr++ = toupper(c);  // make upper case
                }
        }
    }
    // On loop exit, *inPtr is '\0'
    if (parenPtr) {
        // Handle unterminated ( comments
        report_gcode_comment(parenPtr);
    }
    *outPtr = '\0';
}

// Executes one line of NUL-terminated G-Code.
// The line may contain whitespace and comments, which are first removed,
// and lower case characters, which are converted to upper case.
// In this function, all units and positions are converted and
// exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine
// coordinates, respectively.
Error gc_execute_line(char* line, uint8_t client) {
    // Step 0 - remove whitespace and comments and convert to upper case
    collapseGCode(line);
#ifdef REPORT_ECHO_LINE_RECEIVED
    report_echo_line_received(line, client);
#endif

    /* -------------------------------------------------------------------------------------
       STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
       updates these modes and commands as the block line is parser and will only be used and
       executed after successful error-checking. The parser block struct also contains a block
       values struct, word tracking variables, and a non-modal commands tracker for the new
       block. This struct contains all of the necessary information to execute the block. */
    memset(&gc_block, 0, sizeof(parser_block_t));                  // Initialize the parser block struct.
    memcpy(&gc_block.modal, &gc_state.modal, sizeof(gc_modal_t));  // Copy current modes
    AxisCommand axis_command = AxisCommand::None;
    uint8_t     axis_0, axis_1, axis_linear;
    CoordIndex  coord_select = CoordIndex::G54;  // Tracks G10 P coordinate selection for execution
    // Initialize bitflag tracking variables for axis indices compatible operations.
    uint8_t axis_words = 0;  // XYZ tracking
    uint8_t ijk_words  = 0;  // IJK tracking
    // Initialize command and value words and parser flags variables.
    uint32_t command_words   = 0;  // Tracks G and M command words. Also used for modal group violations.
    uint32_t value_words     = 0;  // Tracks value words.
    uint8_t  gc_parser_flags = GCParserNone;
    auto     n_axis          = number_axis->get();
    float    coord_data[MAX_N_AXIS];  // Used by WCO-related commands
    uint8_t  pValue;                  // Integer value of P word

    // Determine if the line is a jogging motion or a normal g-code block.
    if (line[0] == '$') {  // NOTE: `$J=` already parsed when passed to this function.
        // Set G1 and G94 enforced modes to ensure accurate error checks.
        gc_parser_flags |= GCParserJogMotion;
        gc_block.modal.motion    = Motion::Linear;
        gc_block.modal.feed_rate = FeedRate::UnitsPerMin;
#ifdef USE_LINE_NUMBERS
        gc_block.values.n = JOG_LINE_NUMBER;  // Initialize default line number reported during jog.
#endif
    }

    /* -------------------------------------------------------------------------------------
步骤2:导入块行中的所有g码字。g码字是一个字母后面跟着
一个数字，可以是一个'G'/'M'命令，也可以设置/分配一个命令值。同时,
执行命令字模态组违反的初始错误检查
为F、N、P、T和S设置负值。*/
    ModalGroup mg_word_bit;  // Bit-value for assigning tracking variables
    uint32_t   bitmask = 0;
    uint8_t    char_counter;
    char       letter;
    float      value;
    uint8_t    int_value = 0;
    uint16_t   mantissa  = 0;
    if (gc_parser_flags & GCParserJogMotion) {
        char_counter = 3;  // Start parsing after `$J=`
    } else {
        char_counter = 0;
    }
    while (line[char_counter] != 0) {  // Loop until no more g-code words in line.
        // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
        letter = line[char_counter];
        if ((letter < 'A') || (letter > 'Z')) {
            FAIL(Error::ExpectedCommandLetter);  // [Expected word letter]
        }
        char_counter++;
        if (!read_float(line, &char_counter, &value)) {
            FAIL(Error::BadNumberFormat);  // [Expected word value]
        }
        //将值转换为更小的uint8显著值和尾数值，用于解析这个单词。
        //注意:尾数乘以100捕获非整数命令值。这更
        //当用于命令时，比NIST的gcode要求x10准确，但不完全准确
        //对于要求整数不超过0.0001的值词足够精确。这应该是
        //一个足够好的妥协并捕获大多数非整数错误。为了使它符合，
        //我们只需要将尾数更改为int16，但这增加了编译后的flash空间。
        //也许以后会更新这个。
        int_value = trunc(value);
        mantissa  = round(100 * (value - int_value));  //计算Gxx的尾数。x命令。
                                                       //注意:必须使用舍入来捕获小的浮点错误。
                                                       //检查是否支持g码字，或由于模态组违反而导致的错误
                                                       //在g-code块中重复。如果ok，则更新命令或记录其值。
        switch (letter) {
            /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
           NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */
            case 'G':
                // Determine 'G' command and its modal group
                switch (int_value) {
                    // Modal Group G0 - non-modal actions
                    case 10:
                        gc_block.non_modal_command = NonModal::SetCoordinateData;
                        if (mantissa == 0) {
                            if (axis_command != AxisCommand::None) {
                                FAIL(Error::GcodeAxisCommandConflict);  // [Axis word/command conflict]
                            }
                            axis_command = AxisCommand::NonModal;
                        }
                        mg_word_bit = ModalGroup::MG0;
                        break;

                    case 28:
                        gc_block.non_modal_command = mantissa ? NonModal::SetHome0 : NonModal::GoHome0;
                        goto check_mantissa;
                    case 30:
                        gc_block.non_modal_command = mantissa ? NonModal::SetHome1 : NonModal::GoHome1;
                        goto check_mantissa;
                    case 92:
                        gc_block.non_modal_command = mantissa ? mantissa == 2 || mantissa == 20 ? NonModal::SetMachinePosition :
                                                                                                  NonModal::ResetCoordinateOffset :
                                                                NonModal::SetCoordinateOffset;

                    check_mantissa:
                        // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
                        // * G43.1 is also an axis command but is not explicitly defined this way.
                        switch (mantissa) {
                            case 0:  // Ignore G28.1, G30.1, and G92.1
                                if (axis_command != AxisCommand::None) {
                                    FAIL(Error::GcodeAxisCommandConflict);  // [Axis word/command conflict]
                                }
                                axis_command = AxisCommand::NonModal;
                                break;
                            case 2:
                                mantissa     = 0;
                                axis_command = AxisCommand::NonModal;
                                break;
                            case 10:
                                mantissa = 0;  // Set to zero to indicate valid non-integer G command.
                                break;
                            case 20:
                                mantissa     = 0;
                                axis_command = AxisCommand::NonModal;
                                break;
                            default:
                                FAIL(Error::GcodeUnsupportedCommand);
                                // not reached
                                break;
                        }
                        mg_word_bit = ModalGroup::MG0;
                        break;
                    case 4:
                        gc_block.non_modal_command = NonModal::Dwell;
                        mg_word_bit                = ModalGroup::MG0;
                        break;
                    case 53:
                        gc_block.non_modal_command = NonModal::AbsoluteOverride;
                        mg_word_bit                = ModalGroup::MG0;
                        break;

                    // Modal Group G1 - motion commands
                    case 0:  // G0 - linear rapid traverse
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::Seek;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 1:  // G1 - linear feedrate move
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::Linear;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 2:  // G2 - clockwise arc
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::CwArc;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 3:  // G3 - counterclockwise arc
                        axis_command          = AxisCommand::MotionMode;
                        gc_block.modal.motion = Motion::CcwArc;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 38:  // G38 - probe
                        //only allow G38 "Probe" commands if a probe pin is defined.
                        if (PROBE_PIN == UNDEFINED_PIN) {
                            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "No probe pin defined");
                            FAIL(Error::GcodeUnsupportedCommand);  // [Unsupported G command]
                        }
                        // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
                        // * G43.1 is also an axis command but is not explicitly defined this way.
                        if (axis_command != AxisCommand::None) {
                            FAIL(Error::GcodeAxisCommandConflict);  // [Axis word/command conflict]
                        }
                        axis_command = AxisCommand::MotionMode;
                        switch (mantissa) {
                            case 20:
                                gc_block.modal.motion = Motion::ProbeToward;
                                break;
                            case 30:
                                gc_block.modal.motion = Motion::ProbeTowardNoError;
                                break;
                            case 40:
                                gc_block.modal.motion = Motion::ProbeAway;
                                break;
                            case 50:
                                gc_block.modal.motion = Motion::ProbeAway;
                                break;
                            default:
                                FAIL(Error::GcodeUnsupportedCommand);
                                break;  // [Unsupported G38.x command]
                        }
                        mantissa    = 0;  // Set to zero to indicate valid non-integer G command.
                        mg_word_bit = ModalGroup::MG1;
                        break;

                    case 80:  // G80 - cancel canned cycle
                        gc_block.modal.motion = Motion::None;
                        mg_word_bit           = ModalGroup::MG1;
                        break;
                    case 17:
                        gc_block.modal.plane_select = Plane::XY;
                        mg_word_bit                 = ModalGroup::MG2;
                        break;
                    case 18:
                        gc_block.modal.plane_select = Plane::ZX;
                        mg_word_bit                 = ModalGroup::MG2;
                        break;
                    case 19:
                        gc_block.modal.plane_select = Plane::YZ;
                        mg_word_bit                 = ModalGroup::MG2;
                        break;
                    case 90:
                        switch (mantissa) {
                            case 0:
                                gc_block.modal.distance = Distance::Absolute;
                                mg_word_bit             = ModalGroup::MG3;
                                break;
                            case 10:
                                FAIL(Error::GcodeUnsupportedCommand);  // [G90.1 not supported]
                                // mg_word_bit = ModalGroup::MG4;
                                // gc_block.modal.distance_arc = ArcDistance::Absolute;
                                break;
                            default:
                                FAIL(Error::GcodeUnsupportedCommand);
                                break;
                        }
                        break;
                    case 91:
                        switch (mantissa) {
                            case 0:
                                gc_block.modal.distance = Distance::Incremental;
                                mg_word_bit             = ModalGroup::MG3;
                                break;
                            case 10:
                                mantissa = 0;  // Set to zero to indicate valid non-integer G command.
                                // Arc incremental mode is the default and only supported mode
                                // gc_block.modal.distance_arc = ArcDistance::Incremental;
                                mg_word_bit = ModalGroup::MG4;
                                break;
                            default:
                                FAIL(Error::GcodeUnsupportedCommand);
                                break;
                        }
                        break;
                    case 93:
                        gc_block.modal.feed_rate = FeedRate::InverseTime;
                        mg_word_bit              = ModalGroup::MG5;
                        break;
                    case 94:
                        gc_block.modal.feed_rate = FeedRate::UnitsPerMin;
                        mg_word_bit              = ModalGroup::MG5;
                        break;
                    case 20:
                        gc_block.modal.units = Units::Inches;
                        mg_word_bit          = ModalGroup::MG6;
                        break;
                    case 21:
                        gc_block.modal.units = Units::Mm;
                        mg_word_bit          = ModalGroup::MG6;
                        break;
                    case 40:
                        // NOTE: Not required since cutter radius compensation is always disabled. Only here
                        // to support G40 commands that often appear in g-code program headers to setup defaults.
                        // gc_block.modal.cutter_comp = CutterCompensation::Disable; // G40
                        mg_word_bit = ModalGroup::MG7;
                        break;
                    case 43:
                    case 49:
                        // NOTE: The NIST g-code standard vaguely states that when a tool length offset is changed,
                        // there cannot be any axis motion or coordinate offsets updated. Meaning G43, G43.1, and G49
                        // all are explicit axis commands, regardless if they require axis words or not.
                        if (axis_command != AxisCommand::None) {
                            FAIL(Error::GcodeAxisCommandConflict);
                        }
                        // [Axis word/command conflict] }
                        axis_command = AxisCommand::ToolLengthOffset;
                        if (int_value == 49) {  // G49
                            gc_block.modal.tool_length = ToolLengthOffset::Cancel;
                        } else if (mantissa == 10) {  // G43.1
                            gc_block.modal.tool_length = ToolLengthOffset::EnableDynamic;
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);  // [Unsupported G43.x command]
                        }
                        mantissa    = 0;  // Set to zero to indicate valid non-integer G command.
                        mg_word_bit = ModalGroup::MG8;
                        break;
                    case 54:
                        gc_block.modal.coord_select = CoordIndex::G54;
                        mg_word_bit                 = ModalGroup::MG12;
                        break;
                    case 55:
                        gc_block.modal.coord_select = CoordIndex::G55;
                        mg_word_bit                 = ModalGroup::MG12;
                        break;
                    case 56:
                        gc_block.modal.coord_select = CoordIndex::G56;
                        mg_word_bit                 = ModalGroup::MG12;
                        break;
                    case 57:
                        gc_block.modal.coord_select = CoordIndex::G57;
                        mg_word_bit                 = ModalGroup::MG12;
                        break;
                    case 58:
                        gc_block.modal.coord_select = CoordIndex::G58;
                        mg_word_bit                 = ModalGroup::MG12;
                        break;
                    case 59:
                        gc_block.modal.coord_select = CoordIndex::G59;
                        mg_word_bit                 = ModalGroup::MG12;
                        break;
                        // NOTE: G59.x are not supported.
                    case 61:
                        if (mantissa != 0) {
                            FAIL(Error::GcodeUnsupportedCommand);  // [G61.1 not supported]
                        }
                        // gc_block.modal.control = ControlMode::ExactPath; // G61
                        mg_word_bit = ModalGroup::MG13;
                        break;
                    case 99:
                        gc_block.non_modal_command = NonModal::SetMachinePosition;
                        axis_command               = AxisCommand::NonModal;
                        break;
                    default:
                        FAIL(Error::GcodeUnsupportedCommand);  // [Unsupported G command]
                }
                if (mantissa > 0) {
                    FAIL(Error::GcodeCommandValueNotInteger);  // [Unsupported or invalid Gxx.x command]
                }
                // Check for more than one command per modal group violations in the current block
                // NOTE: Variable 'mg_word_bit' is always assigned, if the command is valid.
                bitmask = bit(mg_word_bit);
                if (bit_istrue(command_words, bitmask)) {
                    FAIL(Error::GcodeModalGroupViolation);
                }
                command_words |= bitmask;
                break;
            case 'M':
                // Determine 'M' command and its modal group
                if (mantissa > 0) {
                    FAIL(Error::GcodeCommandValueNotInteger);  // [No Mxx.x commands]
                }
                switch (int_value) {
                    case 0:
                        // M0 - Pause
                        gc_block.modal.program_flow = ProgramFlow::Paused;
                        mg_word_bit                 = ModalGroup::MM4;
                        break;
                    case 1:
                        // M1 - Optional Stop not supported
                        mg_word_bit = ModalGroup::MM4;
                        break;
                    case 2:
                        // M2 - Stop
                        gc_block.modal.program_flow = ProgramFlow::CompletedM2;
                        mg_word_bit                 = ModalGroup::MM4;
                        break;
                    case 30:
                        // M30 - End
                        gc_block.modal.program_flow = ProgramFlow::CompletedM30;
                        mg_word_bit                 = ModalGroup::MM4;
                        break;
                    case 3:
                    case 4:
                    case 5:
                        switch (int_value) {
                            case 3:
                                gc_block.modal.spindle = SpindleState::Cw;
                                break;
                            case 4:  // Supported if SPINDLE_DIR_PIN is defined or laser mode is on.
                                if (spindle->is_reversable || spindle->inLaserMode()) {
                                    gc_block.modal.spindle = SpindleState::Ccw;
                                } else {
                                    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "M4 requires laser mode or a reversable spindle");
                                    FAIL(Error::GcodeUnsupportedCommand);
                                }
                                break;
                            case 5:
                                gc_block.modal.spindle = SpindleState::Disable;
                                break;
                        }
                        mg_word_bit = ModalGroup::MM7;
                        break;
                    case 6:  // tool change
                        gc_block.modal.tool_change = ToolChange::Enable;
                        //user_tool_change(gc_state.tool);
                        mg_word_bit = ModalGroup::MM6;
                        break;
                    case 7:
                    case 8:
                    case 9:
                        switch (int_value) {
#ifdef COOLANT_MIST_PIN
                            case 7:
                                gc_block.coolant = GCodeCoolant::M7;
                                break;
#endif
#ifdef COOLANT_FLOOD_PIN
                            case 8:
                                gc_block.coolant = GCodeCoolant::M8;
                                break;
#endif
                            case 9:
                                gc_block.coolant = GCodeCoolant::M9;
                                break;
                        }
                        mg_word_bit = ModalGroup::MM8;
                        break;
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                    case 56:
                        gc_block.modal.override = Override::ParkingMotion;
                        mg_word_bit             = ModalGroup::MM9;
                        break;
#endif
                    case 62:
                        gc_block.modal.io_control = IoControl::DigitalOnSync;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    case 63:
                        gc_block.modal.io_control = IoControl::DigitalOffSync;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    case 64:
                        gc_block.modal.io_control = IoControl::DigitalOnImmediate;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    case 65:
                        gc_block.modal.io_control = IoControl::DigitalOffImmediate;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    case 67:
                        gc_block.modal.io_control = IoControl::SetAnalogSync;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    case 68:
                        gc_block.modal.io_control = IoControl::SetAnalogImmediate;
                        mg_word_bit               = ModalGroup::MM10;
                        break;
                    default:
                        FAIL(Error::GcodeUnsupportedCommand);  // [Unsupported M command]
                }
                // Check for more than one command per modal group violations in the current block
                // NOTE: Variable 'mg_word_bit' is always assigned, if the command is valid.
                bitmask = bit(mg_word_bit);
                if (bit_istrue(command_words, bitmask)) {
                    FAIL(Error::GcodeModalGroupViolation);
                }
                command_words |= bitmask;
                break;
            // NOTE: All remaining letters assign values.
            default:
                /*非命令字:这个初始解析阶段只检查其余的重复
合法的g码词并存储它们的值。稍后执行错误检查
单词(I,J,K,L,P,R)有多重含义和/或取决于发出的命令。*/
                GCodeWord axis_word_bit;
                switch (letter) {
                    case 'A':
                        if (n_axis > A_AXIS) {
                            axis_word_bit               = GCodeWord::A;
                            gc_block.values.xyz[A_AXIS] = value;
                            axis_words |= bit(A_AXIS);
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        break;
                    case 'B':
                        if (n_axis > B_AXIS) {
                            axis_word_bit               = GCodeWord::B;
                            gc_block.values.xyz[B_AXIS] = value;
                            axis_words |= bit(B_AXIS);
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        break;
                    case 'C':
                        if (n_axis > C_AXIS) {
                            axis_word_bit               = GCodeWord::C;
                            gc_block.values.xyz[C_AXIS] = value;
                            axis_words |= bit(C_AXIS);
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        break;
                    // case 'D': // Not supported
                    case 'E':
                        axis_word_bit     = GCodeWord::E;
                        gc_block.values.e = int_value;
                        //grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "E %d", gc_block.values.e);
                        break;
                    case 'F':
                        axis_word_bit     = GCodeWord::F;
                        gc_block.values.f = value;
                        break;
                    // case 'H': // Not supported
                    case 'I':
                        axis_word_bit               = GCodeWord::I;
                        gc_block.values.ijk[X_AXIS] = value;
                        ijk_words |= bit(X_AXIS);
                        break;
                    case 'J':
                        axis_word_bit               = GCodeWord::J;
                        gc_block.values.ijk[Y_AXIS] = value;
                        ijk_words |= bit(Y_AXIS);
                        break;
                    case 'K':
                        axis_word_bit               = GCodeWord::K;
                        gc_block.values.ijk[Z_AXIS] = value;
                        ijk_words |= bit(Z_AXIS);
                        break;
                    case 'L':
                        axis_word_bit     = GCodeWord::L;
                        gc_block.values.l = int_value;
                        break;
                    case 'N':
                        axis_word_bit     = GCodeWord::N;
                        gc_block.values.n = trunc(value);
                        break;
                    case 'P':
                        axis_word_bit     = GCodeWord::P;
                        gc_block.values.p = value;
                        break;
                    case 'Q':
                        axis_word_bit     = GCodeWord::Q;
                        gc_block.values.q = value;
                        //grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "Q %2.2f", value);
                        break;
                    case 'R':
                        axis_word_bit     = GCodeWord::R;
                        gc_block.values.r = value;
                        break;
                    case 'S':
                        axis_word_bit     = GCodeWord::S;
                        gc_block.values.s = value;
                        break;
                    case 'T':
                        axis_word_bit = GCodeWord::T;
                        if (value > MaxToolNumber) {
                            FAIL(Error::GcodeMaxValueExceeded);
                        }
                        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Tool No: %d", int_value);
                        gc_state.tool = int_value;
                        break;
                    case 'X':
                        if (n_axis > X_AXIS) {
                            axis_word_bit               = GCodeWord::X;
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>value X:%f",
                        //    value);
                            gc_block.values.xyz[X_AXIS] = value;
                            axis_words |= bit(X_AXIS);

                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        break;
                    case 'Y':
                        if (n_axis > Y_AXIS) {
                            axis_word_bit               = GCodeWord::Y;
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>value Y:%f",
                        //    value);
                            gc_block.values.xyz[Y_AXIS] = value;
                            axis_words |= bit(Y_AXIS);
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        break;
                    case 'Z':
                        if (n_axis > Z_AXIS) {
                            axis_word_bit               = GCodeWord::Z;
                            gc_block.values.xyz[Z_AXIS] = value;
                            axis_words |= bit(Z_AXIS);
                        } else {
                            FAIL(Error::GcodeUnsupportedCommand);
                        }
                        break;
                    case 'W':
                        axis_word_bit         = GCodeWord::W;
                        gc_block.values.width = value;

                        break;
                    default:
                        FAIL(Error::GcodeUnsupportedCommand);
                }
                // NOTE: Variable 'axis_word_bit' is always assigned, if the non-command letter is valid.
                uint32_t bitmask = bit(axis_word_bit);
                if (bit_istrue(value_words, bitmask)) {
                    FAIL(Error::GcodeWordRepeated);  // [Word repeated]
                }
                // Check for invalid negative values for words F, N, P, T, and S.
                // NOTE: Negative value check is done here simply for code-efficiency.
                if (bitmask & (bit(GCodeWord::F) | bit(GCodeWord::N) | bit(GCodeWord::P) | bit(GCodeWord::T) | bit(GCodeWord::S) |
                               bit(GCodeWord::W))) {
                    if (value < 0.0) {
                        FAIL(Error::NegativeValue);  // [Word value cannot be negative]
                    }
                }
                value_words |= bitmask;  // Flag to indicate parameter assigned.
        }
    }
    //解析完成！
    /*----------------------------------------------------------------------------------
第 3 步：错误检查在此块中传递的所有命令和值。此步骤确保所有
命令对执行有效，并尽可能遵循 NIST 标准。
如果发现错误，此块中的所有命令和值都将被转储并且不会更新
活动系统 g 代码模式。如果块没问题，活动系统 g 代码模式将为
根据这个块的命令更新，并发出信号让它被执行。
此外，我们必须根据解析的设置的模式预先转换所有传递的值
堵塞。有许多错误检查需要目标信息，这些信息只能是
如果我们转换这些值并结合错误检查，则可以准确计算。
这将下一个执行步骤归为仅更新系统 g 代码模式和
按顺序执行编程的动作。执行步骤不需要任何
转换计算，并且只需要执行所需的最少检查。
*/
    /*注意：此时，g 代码块已被解析，可以释放块行。
注意：在未来的某个时候，也有可能分解第 2 步，以允许分段
在每个单词的基础上解析块，而不是整个块。这可以删除
需要为整个块维护一个大字符串变量并释放一些内存。
为此，只需保留步骤 1 中的所有数据，例如新块
数据结构、模态组和值位标志跟踪变量以及轴数组索引
兼容变量此数据包含错误检查所需的所有信息
当接收到 EOL 字符时，新的 g 代码块。但是，这会破坏 Grbl 的启动
目前的工作方式，需要进行一些重构以使其兼容。
*/
    //[0。非特定/常见错误检查和杂项设置]：
    //确定隐式轴命令条件。轴的话已经传了，但是没有明确的轴
    //命令已发送。如果是这样，将轴命令设置为当前运动模式。
    if (axis_words) {
        if (axis_command == AxisCommand::None) {
            axis_command = AxisCommand::MotionMode;  // Assign implicit motion-mode
        }
    }
    // Check for valid line number N value.
    if (bit_istrue(value_words, bit(GCodeWord::N))) {
        // Line number value cannot be less than zero (done) or greater than max line number.
        if (gc_block.values.n > MaxLineNumber) {
            FAIL(Error::GcodeInvalidLineNumber);  // [Exceeds max line number]
        }
    }
    //bit_false(value_words,bit(GCodeWord::N)); //注意：单义值词。在错误检查结束时设置。
    //在错误检查结束时跟踪未使用的单词。
    //注意：单义值词在错误检查结束时被一次性全部删除，因为
    //它们总是在存在时使用。这样做是为了节省几个字节的闪存。为了清楚起见，
    //单义值词在使用时可能会被删除。此外，轴词在
    //同样的方法。如果有显式/隐式轴命令，则始终使用 XYZ 字，并且
    //在错误检查结束时被删除。
    //[1。评论]：不支持味精。由协议执行的注释处理。
    //[2。设置进给速率模式 ]：G93 F 字丢失，G1、G2/3 激活，隐式或显式。进给率
    //从 G93 切换到 G94 后未定义。
    //注意：对于点动，忽略先前的进给速率模式。执行 G94 并检查所需的 F 字。
    if (gc_parser_flags & GCParserJogMotion) {
        if (bit_isfalse(value_words, bit(GCodeWord::F))) {
            FAIL(Error::GcodeUndefinedFeedRate);
        }
        if (gc_block.modal.units == Units::Inches) {
            gc_block.values.f *= MM_PER_INCH;
        }
    } else {
        if (gc_block.modal.feed_rate == FeedRate::InverseTime) {  // = G93
            // NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
            if (axis_command == AxisCommand::MotionMode) {
                if ((gc_block.modal.motion != Motion::None) || (gc_block.modal.motion != Motion::Seek)) {
                    if (bit_isfalse(value_words, bit(GCodeWord::F))) {
                        FAIL(Error::GcodeUndefinedFeedRate);  // [F word missing]
                    }
                }
            }
            //注意：从 G94 切换到 G93 后检查要传递的 F 字似乎是多余的。我们会
            //如果每次进给率值总是重置为零且未定义，则完成完全相同的事情
            //反时限块，因为使用这个值的命令已经执行了未定义的检查。这个会
            //还允许在此开关之后执行其他命令，而不是不必要地出错。这段代码是
            //结合上面的进给率模式和下面设置的进给率错误检查。
            //[3。设定进给速度]：F为负（完成。）
            //-在反时限模式下：始终在块完成之前和之后将进给速率值隐式归零。
            //注意：如果处于 G93 模式或从 G94 切换到 G93 模式，只需将 F 值保持为初始化零或传递的 F 字
            //块中的值。如果没有通过需要进给速率的运动命令传递 F 字，则会出错
            //在运动模式下进行错误检查。但是，如果没有通过需要
            //一个进给率，我们简单地继续前进，状态进给率值更新为零并保持未定义状态。
        } else {  // = G94
            // - In units per mm mode: If F word passed, ensure value is in mm/min, otherwise push last state value.
            if (gc_state.modal.feed_rate == FeedRate::UnitsPerMin) {  // Last state is also G94
                if (bit_istrue(value_words, bit(GCodeWord::F))) {
                    if (gc_block.modal.units == Units::Inches) {
                        gc_block.values.f *= MM_PER_INCH;
                    }
                } else {
                    gc_block.values.f = gc_state.feed_rate;  // Push last state feed rate
                }
            }  // Else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.
        }
    }
    // bit_false(value_words,bit(GCodeWord::F)); // NOTE: Single-meaning value word. Set at end of error-checking.
    // [4. Set spindle speed ]: S is negative (done.)
    if (bit_isfalse(value_words, bit(GCodeWord::S))) {
        gc_block.values.s = gc_state.spindle_speed;
        // bit_false(value_words,bit(GCodeWord::S)); // NOTE: Single-meaning value word. Set at end of error-checking.
        // [5. Select tool ]: NOT SUPPORTED. Only tracks value. T is negative (done.) Not an integer. Greater than max tool value.
        // bit_false(value_words,bit(GCodeWord::T)); // NOTE: Single-meaning value word. Set at end of error-checking.
        // [6. Change tool ]: N/A
        // [7. Spindle control ]: N/A
        // [8. Coolant control ]: N/A
        // [9. Enable/disable feed rate or spindle overrides ]: NOT SUPPORTED.
    }
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    if (bit_istrue(command_words, bit(ModalGroup::MM9))) {  // Already set as enabled in parser.
        if (bit_istrue(value_words, bit(GCodeWord::P))) {
            if (gc_block.values.p == 0.0) {
                gc_block.modal.override = Override::Disabled;
            }
            bit_false(value_words, bit(GCodeWord::P));
        }
    }
#endif
    // [10. Dwell ]: P value missing. P is negative (done.) NOTE: See below.
    if (gc_block.non_modal_command == NonModal::Dwell) {
        if (bit_isfalse(value_words, bit(GCodeWord::P))) {
            FAIL(Error::GcodeValueWordMissing);  // [P word missing]
        }
        bit_false(value_words, bit(GCodeWord::P));
    }
    if ((gc_block.modal.io_control == IoControl::DigitalOnSync) || (gc_block.modal.io_control == IoControl::DigitalOffSync) ||
        (gc_block.modal.io_control == IoControl::DigitalOnImmediate) || (gc_block.modal.io_control == IoControl::DigitalOffImmediate)) {
        if (bit_isfalse(value_words, bit(GCodeWord::P))) {
            FAIL(Error::GcodeValueWordMissing);  // [P word missing]
        }
        bit_false(value_words, bit(GCodeWord::P));
    }
    if ((gc_block.modal.io_control == IoControl::SetAnalogSync) || (gc_block.modal.io_control == IoControl::SetAnalogImmediate)) {
        if (bit_isfalse(value_words, bit(GCodeWord::E)) || bit_isfalse(value_words, bit(GCodeWord::Q))) {
            FAIL(Error::GcodeValueWordMissing);
        }
        bit_false(value_words, bit(GCodeWord::E));
        bit_false(value_words, bit(GCodeWord::Q));
    }
    // [11. Set active plane ]: N/A
    switch (gc_block.modal.plane_select) {
        case Plane::XY:
            axis_0      = X_AXIS;
            axis_1      = Y_AXIS;
            axis_linear = Z_AXIS;
            break;
        case Plane::ZX:
            axis_0      = Z_AXIS;
            axis_1      = X_AXIS;
            axis_linear = Y_AXIS;
            break;
        default:  // case Plane::YZ:
            axis_0      = Y_AXIS;
            axis_1      = Z_AXIS;
            axis_linear = X_AXIS;
    }

    // [12. Set length units ]: N/A
    // Pre-convert XYZ coordinate values to millimeters, if applicable.
    uint8_t idx;
    if (gc_block.modal.units == Units::Inches) {
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>880 Inches ");
        for (idx = 0; idx < n_axis; idx++) {  // Axes indices are consistent, so loop may be used.
            if (bit_istrue(axis_words, bit(idx))) {
                gc_block.values.xyz[idx] *= MM_PER_INCH;
            }
        }
    }

    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED. Error, if enabled while G53 is active.
    // [G40 Errors]: G2/3 arc is programmed after a G40. The linear move after disabling is less than tool diameter.
    //   NOTE: Since cutter radius compensation is never enabled, these G40 errors don't apply. Grbl supports G40
    //   only for the purpose to not error when G40 is sent with a g-code program header to setup the default modes.
    // [14. Cutter length compensation ]: G43 NOT SUPPORTED, but G43.1 and G49 are.
    // [G43.1 Errors]: Motion command in same line.
    //   NOTE: Although not explicitly stated so, G43.1 should be applied to only one valid
    //   axis that is configured (in config.h). There should be an error if the configured axis
    //   is absent or if any of the other axis words are present.
    if (axis_command == AxisCommand::ToolLengthOffset) {  // Indicates called in block.
        if (gc_block.modal.tool_length == ToolLengthOffset::EnableDynamic) {
            if (axis_words ^ bit(TOOL_LENGTH_OFFSET_AXIS)) {
                FAIL(Error::GcodeG43DynamicAxisError);
            }
        }
    }
    //(15。坐标系统选择]:*N/A。错误，如果刀具半径补偿是活动的。
    // TODO:读取坐标数据时可能需要缓冲区同步
    //是活动的。读操作会暂时暂停处理器，并可能导致罕见的崩溃。为
    //未来版本的处理器有足够的内存，所有的坐标数据应该被存储
    //在内存中，只有在没有周期活动时才写入非易失性存储器。
    float block_coord_system[MAX_N_AXIS];
    memcpy(block_coord_system, gc_state.coord_system, sizeof(gc_state.coord_system));
    if (bit_istrue(command_words, bit(ModalGroup::MG12))) {  // Check if called in block
        // This error probably cannot happen because preceding code sets
        // gc_block.modal.coord_select only to specific supported values
        if (gc_block.modal.coord_select >= CoordIndex::NWCSystems) {
            FAIL(Error::GcodeUnsupportedCoordSys);  // [Greater than N sys]
        }
        if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
            coords[gc_block.modal.coord_select]->get(block_coord_system);
        }
    }
    //[16。设置路径控制模式]：N/A。只有 G61。不支持 G61.1 和 G64。
    //[17。设置距离模式]：N/A。只有 G91.1。不支持 G90.1。
    //[18。设置缩回模式]：不支持。
    //[19。剩余的非模态动作]：选中转到预定义位置，设置 G10，或设置轴偏移。
    //注意：我们需要将使用轴字的非模态命令（G10/G28/G30/G92）分开，因为这些
    //命令都以不同的方式对待轴词。 G10 作为绝对偏移或计算当前位置
    //轴值，G92类似于G10 L20，G28/30作为观察的中间目标位置
    //所有当前坐标系和 G92 偏移量。

    switch (gc_block.non_modal_command) {
        case NonModal::SetCoordinateData:
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>934 SetCoordinateData ");
            // [G10 Errors]: L missing and is not 2 or 20. P word missing. (Negative P value done.)
            // [G10 L2 Errors]: R word NOT SUPPORTED. P value not 0 to nCoordSys(max 9). Axis words missing.
            // [G10 L20 Errors]: P must be 0 to nCoordSys(max 9). Axis words missing.
            if (!axis_words) {
                FAIL(Error::GcodeNoAxisWords)
            };  // [No axis words]
            if (bit_isfalse(value_words, (bit(GCodeWord::P) | bit(GCodeWord::L)))) {
                FAIL(Error::GcodeValueWordMissing);  // [P/L word missing]
            }
            if (gc_block.values.l != 20) {
                if (gc_block.values.l == 2) {
                    if (bit_istrue(value_words, bit(GCodeWord::R))) {
                        FAIL(Error::GcodeUnsupportedCommand);  // [G10 L2 R not supported]
                    }
                } else {
                    FAIL(Error::GcodeUnsupportedCommand);  // [Unsupported L]
                }
            }
            //选择基于P字的坐标系
            pValue = trunc(gc_block.values.p);  //将p值转换为整数
            if (pValue > 0) {
                // P1表示G54, P2表示G55，等等
                coord_select = static_cast<CoordIndex>(pValue - 1 + CoordIndex::G54);
            } else {
                // P0表示使用当前选择的系统
                coord_select = gc_block.modal.coord_select;
            }
            if (coord_select >= CoordIndex::NWCSystems) {
                FAIL(Error::GcodeUnsupportedCoordSys);  //[大于N个sys]
            }
            bit_false(value_words, (bit(GCodeWord::L) | bit(GCodeWord::P)));
            coords[coord_select]->get(coord_data);

            //预计算坐标数据变化。
            for (idx = 0; idx < n_axis; idx++) {  //坐标轴索引是一致的，因此可以使用loop。
                //更新只在块中定义的轴。总是在机器坐标中。可以更改非活动系统。
                if (bit_istrue(axis_words, bit(idx))) {
                    if (gc_block.values.l == 20) {
                        // L20:用已编程值更新当前位置的坐标系轴(用修饰器)
                        // WPos = MPos - WCS - G92 - TLO -> WCS = MPos - G92 - TLO - WPos
                        coord_data[idx] =
                            gc_block.values.xyz[idx];  // gc_state.position[idx] - gc_state.coord_offset[idx] - gc_block.values.xyz[idx];

                        if (idx == TOOL_LENGTH_OFFSET_AXIS) {
                            coord_data[idx] -= gc_state.tool_length_offset;
                            //       grbl_msg_sendf(CLIENT_SERIAL,
                            //    MsgLevel::Warning,
                            //    "TOOL_LENGTH_OFFSET_AXIS:%d, Y:%d  tool_length_offset:%f",
                            //    TOOL_LENGTH_OFFSET_AXIS,
                            //    idx,gc_state.tool_length_offset);
                        }
                    } else {
                        // L2:将坐标系统轴更新为已编程值。
                        coord_data[idx] = gc_block.values.xyz[idx];
                    }
                    // grbl_msg_sendf(CLIENT_SERIAL,
                    //        MsgLevel::Warning,
                    //        "end coord_data[idx]:%f",
                    //        coord_data[idx]);
                }  //否则，保持当前存储值。
            }
            break;
        case NonModal::SetCoordinateOffset:
            // [G92 Errors]: No axis words.
            if (!axis_words) {
                FAIL(Error::GcodeNoAxisWords);  // [No axis words]
            }
            //更新仅在块中定义的轴。将当前系统偏移到定义值。什么时候不更新
            //激活的坐标系被选中，但仍然有效，除非 G92.1 禁用它。
            for (idx = 0; idx < n_axis; idx++) {  // Axes indices are consistent, so loop may be used.
                if (bit_istrue(axis_words, bit(idx))) {
                    // WPos = MPos - WCS - G92 - TLO  ->  G92 = MPos - WCS - TLO - WPos
                    gc_block.values.xyz[idx] = gc_state.position[idx] - block_coord_system[idx] - gc_block.values.xyz[idx];
                    if (idx == TOOL_LENGTH_OFFSET_AXIS) {
                        gc_block.values.xyz[idx] -= gc_state.tool_length_offset;
                    }
                } else {
                    gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
                }
            }
            break;
        case NonModal::SetMachinePosition: {
            //机器宽度
            if (value_words & bit(GCodeWord::W)) {  // Arc Radius Mode
                bit_false(value_words, bit(GCodeWord::W));
                //    auto setting = new FloatSetting(GRBL, WG, makeGrblName(axis, 130), makename(def->name, "MaxTravel"), gc_block.values.width, 0, 100000.0);
                //    setting->setAxis(X_AXIS);
                //    axis_settings[X_AXIS]->max_travel = setting;
                char strval[32];
                (void)sprintf(strval, "%.3f", gc_block.values.width);
                axis_settings[X_AXIS]->max_travel->setStringValue(strval);
            }
            //  float MPosition[MAX_N_AXIS];
            //  MPosition[X_AXIS]  =0;
            //  MPosition[Y_AXIS] = 0;
            //  MPosition[Z_AXIS] = 0;
            //设置线长
            float A = gc_block.values.xyz[X_AXIS];  //+ (machine_width / 2);
            float B = gc_block.values.xyz[Y_AXIS];  //+ (machine_width / 2);
            if (A && B) {
    // float rivetA[n_axis];
    // rivetA[X_AXIS] = 0;
    // rivetA[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
    // float rivetB[n_axis];
    // rivetB[X_AXIS] = axis_settings[X_AXIS]->max_travel->get();
    // rivetB[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
                //左上角铆钉A--0;600
                float* rivetA=sys_get_rivetA();
                //右上角铆钉B--700;600
                float* rivetB=sys_get_rivetB();
                //  grbl_msg_sendf(CLIENT_SERIAL,
                //                MsgLevel::Warning,
                //                "SetMachinePosition   A:%f, B:%f, machine_width:%f , cA:%f , cB:%f",
                //                A,
                //                B,
                //                machine_width,(pow(machine_width, 2.0f) - pow(B, 2.0f) + pow(A, 2.0f)) / (machine_width * 2.0f),sqrt(pow(A, 2.0f) - pow((pow(machine_width, 2.0f) - pow(B, 2.0f) + pow(A, 2.0f)) / (machine_width * 2.0f), 2.0f)));
                //机器位置
                float MPosition[MAX_N_AXIS];
                
                Cal3rdPoint(MPosition,rivetA,rivetB,A,B);
                //（铆钉B的x轴的2次方-机器右线长度的2次方+左线长度的2次方）/ 铆钉B的x轴的2次方=机器X坐标
                // MPosition[X_AXIS] = (pow(rivetB[X_AXIS], 2.0f) - pow(B, 2.0f) + pow(A, 2.0f)) / (rivetB[X_AXIS] * 2.0f);
                // // 机器左线长度的2次方-机器X坐标的2次方的平方根 = 机器Y坐标
                // MPosition[Y_AXIS] = sqrt(pow(A, 2.0f) - pow(MPosition[X_AXIS], 2.0f));
                //机器Z坐标
                MPosition[Z_AXIS] = gc_state.position[Z_AXIS];

                char strvalX[32];
                (void)sprintf(strvalX, "%.3f", MPosition[X_AXIS]);
                char strvalY[32];
                (void)sprintf(strvalY, "%.3f", MPosition[Y_AXIS]);
                //存储原点、当前点信息至eerom
                axis_settings[X_AXIS]->home_mpos->setStringValue(strvalX);
                axis_settings[X_AXIS]->run_current->setStringValue(strvalX);
                axis_settings[X_AXIS]->hold_current->setStringValue(strvalX);

                axis_settings[Y_AXIS]->home_mpos->setStringValue(strvalY);
                axis_settings[Y_AXIS]->run_current->setStringValue(strvalY);
                axis_settings[Y_AXIS]->hold_current->setStringValue(strvalY);
                //   grbl_msg_sendf(CLIENT_SERIAL,
                //                MsgLevel::Warning,
                //                "SetMachinePosition MPosition X:%f, Y:%f",
                //                MPosition[X_AXIS],
                //                MPosition[Y_AXIS]);
                //new   memcpy(gc_state.position, MPosition, sizeof(MPosition));

                //memcpy(gc_state.coord_offset,MPosition, sizeof(MPosition));
                //memcpy(gc_state.coord_system,MPosition, sizeof(MPosition));
                //转换坐标到所需步数
                //（机器左线长度-(铆钉X坐标/2)）*机器每步长度
              
                //memcpy(gc_state.position, sys_position, sizeof(sys_position));
                //memcpy(gc_block.values.xyz, gc_state.position, sizeof(gc_state.position)); //
                
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "1 sys_position now X:%d, Y:%d, Z:%d",
                           sys_position[X_AXIS],
                           sys_position[Y_AXIS],
                           sys_position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "1 sys_probe_position X:%d, Y:%d, Z:%d",
                           sys_probe_position[X_AXIS],
                           sys_probe_position[Y_AXIS],
                           sys_probe_position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "1 gc_state.position X:%f, Y:%f, Z:%f",
                           gc_state.position[X_AXIS],
                           gc_state.position[Y_AXIS],
                           gc_state.position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "1 gc_block.values.xyz X:%f, Y:%f, Z:%f",
                           gc_block.values.xyz[X_AXIS],
                           gc_block.values.xyz[Y_AXIS],
                           gc_block.values.xyz[Z_AXIS]);
                auto n_axis = number_axis->get();
                for (int idx = 0; idx < n_axis; idx++) {
                    gc_state.position[idx] = MPosition[idx];//可能跟它有关  gc_state.position[idx] = MPosition[idx];时；每次设置坐标漂移量会变化，设置相同坐标时又无漂移现象；不赋值时仅开机后设置位置漂移一次
                    gc_state.coord_offset[idx] = 0;
                    //gc_state.coord_system[idx] = 0;//MPosition[idx];
                    gc_block.values.xyz[idx] = MPosition[idx];
                    //coord_data[idx]=MPosition[idx];//fk//
                }    
                sys_position[X_AXIS] = (A //- (rivetB[X_AXIS] / 2)
                ) * axis_settings[X_AXIS]->steps_per_mm->get();
                //（机器右线长度-(铆钉X坐标/2)）*机器每步长度
                sys_position[Y_AXIS] = (B //- (rivetB[X_AXIS] / 2)
                ) * axis_settings[Y_AXIS]->steps_per_mm->get();
                
                sys_probe_position[X_AXIS] = (A //- (rivetB[X_AXIS] / 2)
                ) * axis_settings[X_AXIS]->steps_per_mm->get();
                //（机器右线长度-(铆钉X坐标/2)）*机器每步长度
                sys_probe_position[Y_AXIS] = (B //- (rivetB[X_AXIS] / 2)
                ) * axis_settings[Y_AXIS]->steps_per_mm->get();
         sys.step_control = {};
                    plan_clear();
                    //
                   // plan_reset();
                    //gc_sync_position();
                    plan_sync_position();  
                    st_reset();
                    //stepper_clear_buff();
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "2 sys_position now X:%d, Y:%d, Z:%d",
                           sys_position[X_AXIS],
                           sys_position[Y_AXIS],
                           sys_position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "2 sys_probe_position X:%d, Y:%d, Z:%d",
                           sys_probe_position[X_AXIS],
                           sys_probe_position[Y_AXIS],
                           sys_probe_position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "2 gc_state.position X:%f, Y:%f, Z:%f",
                           gc_state.position[X_AXIS],
                           gc_state.position[Y_AXIS],
                           gc_state.position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "2 gc_block.values.xyz X:%f, Y:%f, Z:%f",
                           gc_block.values.xyz[X_AXIS],
                           gc_block.values.xyz[Y_AXIS],
                           gc_block.values.xyz[Z_AXIS]);
    plan_line_data_t  plan_data;
    plan_line_data_t* pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t));  // 零pl_data结构
    // Initialize planner data to current spindle and coolant modal state.
        pl_data->spindle_speed  = gc_state.spindle_speed;
        pl_data->spindle        = gc_state.modal.spindle;
        pl_data->coolant        = gc_state.modal.coolant;
        gc_block.values.f=8000;
        bool  cancelledInflight = false;
        Error status            = jog_execute(pl_data, &gc_block, &cancelledInflight);
        
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "3 sys_position now X:%d, Y:%d, Z:%d",
                           sys_position[X_AXIS],
                           sys_position[Y_AXIS],
                           sys_position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "3 sys_probe_position X:%d, Y:%d, Z:%d",
                           sys_probe_position[X_AXIS],
                           sys_probe_position[Y_AXIS],
                           sys_probe_position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "3 gc_state.position X:%f, Y:%f, Z:%f",
                           gc_state.position[X_AXIS],
                           gc_state.position[Y_AXIS],
                           gc_state.position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "3 gc_block.values.xyz X:%f, Y:%f, Z:%f",
                           gc_block.values.xyz[X_AXIS],
                           gc_block.values.xyz[Y_AXIS],
                           gc_block.values.xyz[Z_AXIS]);
        // if (status == Error::Ok && !cancelledInflight) {
        //     memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
            
        // }   
            
                //  memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
                //  plan_clear();
                //  stepper_clear_buff();
                //  st_reset();
                //  stepper_init();
                //gc_state.position是步进电机基坐标，更改会导致漂移，因为它要转到对应位置，它和sys_position并不是一致的，但是是同步变化的
                //plan_sync_position();
                //planner_recalculate();//已经验证 无效
     //memset(&pl, 0, sizeof(planner_t));  // Clear planner struct
    // plan_reset_buffer();//正在验证 +plan_sync_position似乎有效
    //  plan_sync_position();//同步计划位置
    //gc_sync_position();//会影响漂移结果 相当于gc_state.position[idx] = MPosition[idx];
                // plan_discard_current_block();//丢弃当前计划块 已经验证 无效
                // plan_cycle_reinitialize() ;//丢弃所有计划 已验证
    //             plan_reset();
    //             // gc_init() ;
                //  
               //   system_flag_wco_change();
            }
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "gc_block.values.xyz X:%f, Y:%f, Z:%f",
                           gc_block.values.xyz[X_AXIS],
                           gc_block.values.xyz[Y_AXIS],
                           gc_block.values.xyz[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "gc_state.position X:%f, Y:%f, Z:%f",
                           gc_state.position[X_AXIS],
                           gc_state.position[Y_AXIS],
                           gc_state.position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "gc_state.coord_system X:%f, Y:%f, Z:%f",
                           gc_state.coord_system[X_AXIS],
                           gc_state.coord_system[Y_AXIS],
                           gc_state.coord_system[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "gc_state.coord_offset X:%f, Y:%f, Z:%f",
                           gc_state.coord_offset[X_AXIS],
                           gc_state.coord_offset[Y_AXIS],
                           gc_state.coord_offset[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "sys_position X:%d, Y:%d, Z:%d",
                           sys_position[X_AXIS],
                           sys_position[Y_AXIS],
                           sys_position[Z_AXIS]);
            grbl_msg_sendf(CLIENT_SERIAL,
                           MsgLevel::Warning,
                           "sys_probe_position X:%d, Y:%d, Z:%d",
                           sys_probe_position[X_AXIS],
                           sys_probe_position[Y_AXIS],
                           sys_probe_position[Z_AXIS]);
            // grbl_msg_sendf(
            //     CLIENT_SERIAL, MsgLevel::Warning, "MPosition X:%f, Y:%f, Z:%f", MPosition[X_AXIS], MPosition[Y_AXIS], MPosition[Z_AXIS]);
            //
            //  //memcpy(sys_position,MPosition, sizeof(MPosition));
            //  memcpy(gc_block.values.xyz,MPosition, sizeof(MPosition));
            //  memcpy(gc_state.position,MPosition, sizeof(MPosition));
            // memcpy(gc_state.coord_offset, MPosition, sizeof(MPosition));
            // memcpy(gc_block.values.xyz, MPosition, sizeof(gc_block.values.xyz));
            // memcpy(gc_state.coord_system, MPosition, sizeof(MPosition));
            // gc_state.coord_offset[X_AXIS]=0;
            // gc_state.coord_offset[Y_AXIS]=0;
            // gc_state.coord_offset[Z_AXIS]=0;
            // gc_block.values.xyz[X_AXIS]=0;
            // gc_block.values.xyz[Y_AXIS]=0;
            // gc_block.values.xyz[Z_AXIS]=0;
            // coord_data[idx] = gc_state.position[idx] - gc_state.coord_offset[idx] - gc_block.values.xyz[idx];
            // memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
            //   grbl_msg_sendf(CLIENT_SERIAL,
            //                            MsgLevel::Warning,
            //                            "SetMachinePosition   gc_block.values.xyz X:%f, Y:%f, Z:%f   gc_state.position X:%f, Y:%f, Z:%f   gc_state.coord_offset X:%f, Y:%f, Z:%f   sys_position X:%f, Y:%f, Z:%f   sys_probe_position X:%f, Y:%f, Z:%f",
            //                            gc_block.values.xyz[X_AXIS],
            //                            gc_block.values.xyz[Y_AXIS],
            //                            gc_block.values.xyz[Z_AXIS],
            //                            gc_state.position[X_AXIS],
            //                            gc_state.position[Y_AXIS],
            //                            gc_state.position[Z_AXIS],
            //                            gc_state.coord_offset[X_AXIS],
            //                            gc_state.coord_offset[Y_AXIS],
            //                            gc_state.coord_offset[Z_AXIS],
            //                            sys_position[X_AXIS],
            //                            sys_position[Y_AXIS],
            //                            sys_position[Z_AXIS],
            //                            sys_probe_position[X_AXIS],
            //                            sys_probe_position[Y_AXIS],
            //                            sys_probe_position[Z_AXIS],
            //                            axis_settings[X_AXIS]->max_travel->get());
        } break;
        default:
            //此时，其余显式轴命令将轴值视为传统的
            //目标位置与坐标系统的偏移量，G92偏移量，绝对覆盖和距离
            //模式应用。这包括运动模式命令。我们现在可以预先计算目标位置。
            //注意:当/如果添加了这个特性，工具偏移量可能会附加到这些转换中。
            if (axis_command != AxisCommand::ToolLengthOffset) {  // TLO block any axis command.
            // grbl_msg_sendf(CLIENT_SERIAL,
            //                MsgLevel::Warning,
            //                ">>1201 jiagongqian ToolLengthOffset");
                if (axis_words) {
                    for (idx = 0; idx < n_axis; idx++) {  // Axes indices are consistent, so loop may be used to save flash space.
                        //检查idx坐标轴是否有值，没有值直接赋当前gc_state.position[idx] 
                        if (bit_isfalse(axis_words, bit(idx))) {
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>1210 gc_block.values.xyz[idx] = gc_state.position[idx]; ");
                            gc_block.values.xyz[idx] = gc_state.position[idx];  // No axis word in block. Keep same axis position.
                        } else {
                            //否则判断是否绝对坐标
                            // Update specified value according to distance mode or ignore if absolute override is active.根据距离模式更新指定值，如果绝对覆盖激活则忽略。
                            // NOTE: G53 is never active with G28/30 since they are in the same modal group.注意:G53永远不会与G28/30一起活动，因为它们在同一个模态组中。
                            if (gc_block.non_modal_command != NonModal::AbsoluteOverride) {
                                // Apply coordinate offsets based on distance mode.应用基于距离模式的坐标偏移。
                                //判断是否为绝对坐标
                                if (gc_block.modal.distance == Distance::Absolute) {
                
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>1221 gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];");
                                    gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];
               
                                    if (idx == TOOL_LENGTH_OFFSET_AXIS) {
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>1227 gc_block.values.xyz[idx] += gc_state.tool_length_offset; ");
                                        gc_block.values.xyz[idx] += gc_state.tool_length_offset;
                                    }
                                } else {  // Incremental mode 相对坐标 增量坐标
                        //      grbl_msg_sendf(CLIENT_SERIAL,
                        //    MsgLevel::Warning,
                        //    ">>1233 gc_block.values.xyz[idx] = gc_state.position[idx]; ");
                                    gc_block.values.xyz[idx] += gc_state.position[idx];
                                }
                            }
                        }
                    }
                }
                // grbl_msg_sendf(CLIENT_SERIAL,
                //            MsgLevel::Warning,
                //            ">>1230 jiagonghou gc_block.values.xyz");
            }
            // Check remaining non-modal commands for errors.
            switch (gc_block.non_modal_command) {
                case NonModal::GoHome0:  // G28
                case NonModal::GoHome1:  // G30
                    // [G28/30 Errors]: Cutter compensation is enabled.
                    // Retreive G28/30 go-home position data (in machine coordinates) from non-volatile storage
                    if (gc_block.non_modal_command == NonModal::GoHome0) {
                        coords[CoordIndex::G28]->get(coord_data);
                    } else {  // == NonModal::GoHome1
                        coords[CoordIndex::G30]->get(coord_data);
                    }
                    if (axis_words) {
                        // Move only the axes specified in secondary move.
                        for (idx = 0; idx < n_axis; idx++) {
                            if (!(axis_words & bit(idx))) {
                                coord_data[idx] = gc_state.position[idx];
                            }
                        }
                    } else {
                        axis_command = AxisCommand::None;  // Set to none if no intermediate motion.
                    }
                    break;
                case NonModal::SetHome0:  // G28.1
                case NonModal::SetHome1:  // G30.1
                    // [G28.1/30.1 Errors]: Cutter compensation is enabled.
                    // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
                    break;
                case NonModal::ResetCoordinateOffset:
                    // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
                    break;
                case NonModal::SetMachinePosition:
                    // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
                    break;
                case NonModal::AbsoluteOverride:
                    // [G53 Errors]: G0 and G1 are not active. Cutter compensation is enabled.
                    // NOTE: All explicit axis word commands are in this modal group. So no implicit check necessary.
                    if (!(gc_block.modal.motion == Motion::Seek || gc_block.modal.motion == Motion::Linear)) {
                        FAIL(Error::GcodeG53InvalidMotionMode);  // [G53 G0/1 not active]
                    }
                    break;
                default:
                    break;
            }
    }
    // [20. Motion modes ]:
    if (gc_block.modal.motion == Motion::None) {
        // [G80 Errors]: Axis word are programmed while G80 is active.
        // NOTE: Even non-modal commands or TLO that use axis words will throw this strict error.
        if (axis_words) {
            FAIL(Error::GcodeAxisWordsExist);  // [No axis words allowed]
        }
        // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or
        // was explicitly commanded in the g-code block.
    } else if (axis_command == AxisCommand::MotionMode) {
        if (gc_block.modal.motion == Motion::Seek) {
            // [G0 Errors]: Axis letter not configured or without real value (done.)
            // Axis words are optional. If missing, set axis command flag to ignore execution.
            if (!axis_words) {
                axis_command = AxisCommand::None;
            }
            // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
            // the value must be positive. In inverse time mode, a positive value must be passed with each block.
        } else {
            // Check if feed rate is defined for the motion modes that require it.
            if (gc_block.values.f == 0.0) {
                FAIL(Error::GcodeUndefinedFeedRate);  // [Feed rate undefined]
            }
            switch (gc_block.modal.motion) {
                case Motion::None:
                    break;  // Feed rate is unnecessary
                case Motion::Seek:
                    break;  // Feed rate is unnecessary
                case Motion::Linear:
                    // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
                    // Axis words are optional. If missing, set axis command flag to ignore execution.
                    if (!axis_words) {
                        axis_command = AxisCommand::None;
                    }
                    break;
                case Motion::CwArc:
                    gc_parser_flags |= GCParserArcIsClockwise;  // No break intentional.
                case Motion::CcwArc:
                    // [G2/3 Errors All-Modes]: Feed rate undefined.
                    // [G2/3 Radius-Mode Errors]: No axis words in selected plane. Target point is same as current.
                    // [G2/3 Offset-Mode Errors]: No axis words and/or offsets in selected plane. The radius to the current
                    //   point and the radius to the target point differs more than 0.002mm (EMC def. 0.5mm OR 0.005mm and 0.1% radius).
                    // [G2/3 Full-Circle-Mode Errors]: NOT SUPPORTED. Axis words exist. No offsets programmed. P must be an integer.
                    // NOTE: Both radius and offsets are required for arc tracing and are pre-computed with the error-checking.
                    if (!axis_words) {
                        FAIL(Error::GcodeNoAxisWords);  // [No axis words]
                    }
                    if (!(axis_words & (bit(axis_0) | bit(axis_1)))) {
                        FAIL(Error::GcodeNoAxisWordsInPlane);  // [No axis words in plane]
                    }
                    // Calculate the change in position along each selected axis
                    float x, y;
                    x = gc_block.values.xyz[axis_0] - gc_state.position[axis_0];  // Delta x between current position and target
                    y = gc_block.values.xyz[axis_1] - gc_state.position[axis_1];  // Delta y between current position and target
                    if (value_words & bit(GCodeWord::R)) {                        // Arc Radius Mode
                        bit_false(value_words, bit(GCodeWord::R));
                        if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) {
                            FAIL(Error::GcodeInvalidTarget);  // [Invalid target]
                        }
                        // Convert radius value to proper units.
                        if (gc_block.modal.units == Units::Inches) {
                            gc_block.values.r *= MM_PER_INCH;
                        }
                        /*  We need to calculate the center of the circle that has the designated radius and passes
                        through both the current position and the target position. This method calculates the following
                        set of equations where [x,y] is the vector from current to target position, d == magnitude of
                        that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
                        the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
                        length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
                        [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

                        d^2 == x^2 + y^2
                        h^2 == r^2 - (d/2)^2
                        i == x/2 - y/d*h
                        j == y/2 + x/d*h

                                                                             O <- [i,j]
                                                                          -  |
                                                                r      -     |
                                                                    -        |
                                                                 -           | h
                                                              -              |
                                                [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                          | <------ d/2 ---->|

                        C - Current position
                        T - Target position
                        O - center of circle that pass through both C and T
                        d - distance from C to T
                        r - designated radius
                        h - distance from center of CT to O

                        Expanding the equations:

                        d -> sqrt(x^2 + y^2)
                        h -> sqrt(4 * r^2 - x^2 - y^2)/2
                        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
                        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

                        Which can be written:

                        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

                        Which we for size and speed reasons optimize to:

                        h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                        i = (x - (y * h_x2_div_d))/2
                        j = (y + (x * h_x2_div_d))/2
                    */
                        // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
                        // than d. If so, the sqrt of a negative number is complex and error out.
                        float h_x2_div_d = 4.0 * gc_block.values.r * gc_block.values.r - x * x - y * y;
                        if (h_x2_div_d < 0) {
                            FAIL(Error::GcodeArcRadiusError);  // [Arc radius error]
                        }
                        // Finish computing h_x2_div_d.
                        h_x2_div_d = -sqrt(h_x2_div_d) / hypot_f(x, y);  // == -(h * 2 / d)
                        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
                        if (gc_block.modal.motion == Motion::CcwArc) {
                            h_x2_div_d = -h_x2_div_d;
                        }
                        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
                       the left hand circle will be generated - when it is negative the right hand circle is generated.

                                                                           T  <-- Target position

                                                                           ^
                                Clockwise circles with this center         |          Clockwise circles with this center will have
                                will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                                 \         |          /
                    center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                                           |
                                                                           |

                                                                           C  <-- Current position
                    */
                        // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
                        // even though it is advised against ever generating such circles in a single line of g-code. By
                        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
                        // travel and thus we get the unadvisably long arcs as prescribed.
                        if (gc_block.values.r < 0) {
                            h_x2_div_d        = -h_x2_div_d;
                            gc_block.values.r = -gc_block.values.r;  // Finished with r. Set to positive for mc_arc
                        }
                        // Complete the operation by calculating the actual center of the arc
                        gc_block.values.ijk[axis_0] = 0.5 * (x - (y * h_x2_div_d));
                        gc_block.values.ijk[axis_1] = 0.5 * (y + (x * h_x2_div_d));
                    } else {  // Arc Center Format Offset Mode
                        if (!(ijk_words & (bit(axis_0) | bit(axis_1)))) {
                            FAIL(Error::GcodeNoOffsetsInPlane);  // [No offsets in plane]
                        }
                        bit_false(value_words, (bit(GCodeWord::I) | bit(GCodeWord::J) | bit(GCodeWord::K)));
                        // Convert IJK values to proper units.
                        if (gc_block.modal.units == Units::Inches) {
                            for (idx = 0; idx < n_axis; idx++) {  // Axes indices are consistent, so loop may be used to save flash space.
                                if (ijk_words & bit(idx)) {
                                    gc_block.values.ijk[idx] *= MM_PER_INCH;
                                }
                            }
                        }
                        // Arc radius from center to target
                        x -= gc_block.values.ijk[axis_0];  // Delta x between circle center and target
                        y -= gc_block.values.ijk[axis_1];  // Delta y between circle center and target
                        float target_r = hypot_f(x, y);
                        // Compute arc radius for mc_arc. Defined from current location to center.
                        gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]);
                        // Compute difference between current location and target radii for final error-checks.
                        float delta_r = fabs(target_r - gc_block.values.r);
                        if (delta_r > 0.005) {
                            if (delta_r > 0.5) {
                                FAIL(Error::GcodeInvalidTarget);  // [Arc definition error] > 0.5mm
                            }
                            if (delta_r > (0.001 * gc_block.values.r)) {
                                FAIL(Error::GcodeInvalidTarget);  // [Arc definition error] > 0.005mm AND 0.1% radius
                            }
                        }
                    }
                    break;
                case Motion::ProbeTowardNoError:
                case Motion::ProbeAwayNoError:
                    gc_parser_flags |= GCParserProbeIsNoError;  // No break intentional.
                case Motion::ProbeToward:
                case Motion::ProbeAway:
                    if ((gc_block.modal.motion == Motion::ProbeAway) || (gc_block.modal.motion == Motion::ProbeAwayNoError)) {
                        gc_parser_flags |= GCParserProbeIsAway;
                    }
                    // [G38 Errors]: Target is same current. No axis words. Cutter compensation is enabled. Feed rate
                    //   is undefined. Probe is triggered. NOTE: Probe check moved to probe cycle. Instead of returning
                    //   an error, it issues an alarm to prevent further motion to the probe. It's also done there to
                    //   allow the planner buffer to empty and move off the probe trigger before another probing cycle.
                    if (!axis_words) {
                        FAIL(Error::GcodeNoAxisWords);  // [No axis words]
                    }
                    if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) {
                        FAIL(Error::GcodeInvalidTarget);  // [Invalid target]
                    }
                    break;
            }
        }
    }
    // [21. Program flow ]: No error checks required.
    // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
    // radius mode, or axis words that aren't used in the block.
    if (gc_parser_flags & GCParserJogMotion) {
        // Jogging only uses the F feed rate and XYZ value words. N is valid, but S and T are invalid.
        bit_false(value_words, (bit(GCodeWord::N) | bit(GCodeWord::F)));
    } else {
        bit_false(value_words,
                  (bit(GCodeWord::N) | bit(GCodeWord::F) | bit(GCodeWord::S) | bit(GCodeWord::T)));  // Remove single-meaning value words.
    }
    if (axis_command != AxisCommand::None) {
        bit_false(value_words,
                  (bit(GCodeWord::X) | bit(GCodeWord::Y) | bit(GCodeWord::Z) | bit(GCodeWord::A) | bit(GCodeWord::B) |
                   bit(GCodeWord::C)));  // Remove axis words.
    }
    if (value_words) {
        FAIL(Error::GcodeUnusedWords);  // [Unused words]
    }
    /* -------------------------------------------------------------------------------------
       STEP 4: EXECUTE!!
       Assumes that all error-checking has been completed and no failure modes exist. We just
       need to update the state and execute the block according to the order-of-execution.
    */
    // Initialize planner data struct for motion blocks.
    plan_line_data_t  plan_data;
    plan_line_data_t* pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t));  // Zero pl_data struct
    // Intercept jog commands and complete error checking for valid jog commands and execute.
    // NOTE: G-code parser state is not updated, except the position to ensure sequential jog
    // targets are computed correctly. The final parser position after a jog is updated in
    // protocol_execute_realtime() when jogging completes or is canceled.
    if (gc_parser_flags & GCParserJogMotion) {
        // Only distance and unit modal commands and G53 absolute override command are allowed.
        // NOTE: Feed rate word and axis word checks have already been performed in STEP 3.
        if (command_words & ~(bit(ModalGroup::MG3) | bit(ModalGroup::MG6) | bit(ModalGroup::MG0))) {
            FAIL(Error::InvalidJogCommand)
        };
        if (!(gc_block.non_modal_command == NonModal::AbsoluteOverride || gc_block.non_modal_command == NonModal::NoAction)) {
            FAIL(Error::InvalidJogCommand);
        }
        // Initialize planner data to current spindle and coolant modal state.
        pl_data->spindle_speed  = gc_state.spindle_speed;
        pl_data->spindle        = gc_state.modal.spindle;
        pl_data->coolant        = gc_state.modal.coolant;
        bool  cancelledInflight = false;
        Error status            = jog_execute(pl_data, &gc_block, &cancelledInflight);
        if (status == Error::Ok && !cancelledInflight) {
            memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
        }
        // JogCancelled is not reported as a GCode error
        return status == Error::JogCancelled ? Error::Ok : status;
    }
    // If in laser mode, setup laser power based on current and past parser conditions.
    if (spindle->inLaserMode()) {
        if (!((gc_block.modal.motion == Motion::Linear) || (gc_block.modal.motion == Motion::CwArc) ||
              (gc_block.modal.motion == Motion::CcwArc))) {
            gc_parser_flags |= GCParserLaserDisable;
        }
        // Any motion mode with axis words is allowed to be passed from a spindle speed update.
        // NOTE: G1 and G0 without axis words sets axis_command to none. G28/30 are intentionally omitted.
        // TODO: Check sync conditions for M3 enabled motions that don't enter the planner. (zero length).
        if (axis_words && (axis_command == AxisCommand::MotionMode)) {
            gc_parser_flags |= GCParserLaserIsMotion;
        } else {
            // M3 constant power laser requires planner syncs to update the laser when changing between
            // a G1/2/3 motion mode state and vice versa when there is no motion in the line.
            if (gc_state.modal.spindle == SpindleState::Cw) {
                if ((gc_state.modal.motion == Motion::Linear) || (gc_state.modal.motion == Motion::CwArc) ||
                    (gc_state.modal.motion == Motion::CcwArc)) {
                    if (bit_istrue(gc_parser_flags, GCParserLaserDisable)) {
                        gc_parser_flags |= GCParserLaserForceSync;  // Change from G1/2/3 motion mode.
                    }
                } else {
                    // When changing to a G1 motion mode without axis words from a non-G1/2/3 motion mode.
                    if (bit_isfalse(gc_parser_flags, GCParserLaserDisable)) {
                        gc_parser_flags |= GCParserLaserForceSync;
                    }
                }
            }
        }
    }
    // [0. Non-specific/common error-checks and miscellaneous setup]:
    // NOTE: If no line number is present, the value is zero.
    gc_state.line_number = gc_block.values.n;
#ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_state.line_number;  // Record data for planner use.
#endif
    // [1. Comments feedback ]:  NOT SUPPORTED
    // [2. Set feed rate mode ]:
    gc_state.modal.feed_rate = gc_block.modal.feed_rate;
    if (gc_state.modal.feed_rate == FeedRate::InverseTime) {
        pl_data->motion.inverseTime = 1;  // Set condition flag for planner use.
    }
    // [3. Set feed rate ]:
    gc_state.feed_rate = gc_block.values.f;   // Always copy this value. See feed rate error-checking.
    pl_data->feed_rate = gc_state.feed_rate;  // Record data for planner use.
    // [4. Set spindle speed ]:
    if ((gc_state.spindle_speed != gc_block.values.s) || bit_istrue(gc_parser_flags, GCParserLaserForceSync)) {
        if (gc_state.modal.spindle != SpindleState::Disable) {
            if (bit_isfalse(gc_parser_flags, GCParserLaserIsMotion)) {
                if (bit_istrue(gc_parser_flags, GCParserLaserDisable)) {
                    spindle->sync(gc_state.modal.spindle, 0);
                } else {
                    spindle->sync(gc_state.modal.spindle, (uint32_t)gc_block.values.s);
                }
            }
        }
        gc_state.spindle_speed = gc_block.values.s;  // Update spindle speed state.
    }
    // NOTE: Pass zero spindle speed for all restricted laser motions.
    if (bit_isfalse(gc_parser_flags, GCParserLaserDisable)) {
        pl_data->spindle_speed = gc_state.spindle_speed;  // Record data for planner use.
    }                                                     // else { pl_data->spindle_speed = 0.0; } // Initialized as zero already.
    // [5. Select tool ]: NOT SUPPORTED. Only tracks tool value.
    //	gc_state.tool = gc_block.values.t;
    // [6. Change tool ]: NOT SUPPORTED
    if (gc_block.modal.tool_change == ToolChange::Enable) {
        user_tool_change(gc_state.tool);
    }
    // [7. Spindle control ]:
    if (gc_state.modal.spindle != gc_block.modal.spindle) {
        // Update spindle control and apply spindle speed when enabling it in this block.
        // NOTE: All spindle state changes are synced, even in laser mode. Also, pl_data,
        // rather than gc_state, is used to manage laser state for non-laser motions.
        spindle->sync(gc_block.modal.spindle, (uint32_t)pl_data->spindle_speed);
        gc_state.modal.spindle = gc_block.modal.spindle;
    }
    pl_data->spindle = gc_state.modal.spindle;
    // [8. Coolant control ]:
    // At most one of M7, M8, M9 can appear in a GCode block, but the overall coolant
    // state can have both mist (M7) and flood (M8) on at once, by issuing M7 and M8
    // in separate blocks.  There is no GCode way to turn them off separately, but
    // you can turn them off simultaneously with M9.  You can turn them off separately
    // with real-time overrides, but that is out of the scope of GCode.
    switch (gc_block.coolant) {
        case GCodeCoolant::None:
            break;
        case GCodeCoolant::M7:
            gc_state.modal.coolant.Mist = 1;
            coolant_sync(gc_state.modal.coolant);
            break;
        case GCodeCoolant::M8:
            gc_state.modal.coolant.Flood = 1;
            coolant_sync(gc_state.modal.coolant);
            break;
        case GCodeCoolant::M9:
            gc_state.modal.coolant = {};
            coolant_sync(gc_state.modal.coolant);
            break;
    }
    pl_data->coolant = gc_state.modal.coolant;  // Set state for planner use.
    // turn on/off an i/o pin
    if ((gc_block.modal.io_control == IoControl::DigitalOnSync) || (gc_block.modal.io_control == IoControl::DigitalOffSync) ||
        (gc_block.modal.io_control == IoControl::DigitalOnImmediate) || (gc_block.modal.io_control == IoControl::DigitalOffImmediate)) {
        if (gc_block.values.p < MaxUserDigitalPin) {
            if ((gc_block.modal.io_control == IoControl::DigitalOnSync) || (gc_block.modal.io_control == IoControl::DigitalOffSync)) {
                protocol_buffer_synchronize();
            }
            bool turnOn = gc_block.modal.io_control == IoControl::DigitalOnSync || gc_block.modal.io_control == IoControl::DigitalOnImmediate;
            if (!sys_set_digital((int)gc_block.values.p, turnOn)) {
                FAIL(Error::PParamMaxExceeded);
            }
        } else {
            FAIL(Error::PParamMaxExceeded);
        }
    }
    if ((gc_block.modal.io_control == IoControl::SetAnalogSync) || (gc_block.modal.io_control == IoControl::SetAnalogImmediate)) {
        if (gc_block.values.e < MaxUserDigitalPin) {
            gc_block.values.q = constrain(gc_block.values.q, 0.0, 100.0);  // force into valid range
            if (gc_block.modal.io_control == IoControl::SetAnalogSync) {
                protocol_buffer_synchronize();
            }
            if (!sys_set_analog((int)gc_block.values.e, gc_block.values.q)) {
                FAIL(Error::PParamMaxExceeded);
            }
        } else {
            FAIL(Error::PParamMaxExceeded);
        }
    }

    // [9. Override control ]: NOT SUPPORTED. Always enabled. Except for a Grbl-only parking control.
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    if (gc_state.modal.override != gc_block.modal.override) {
        gc_state.modal.override = gc_block.modal.override;
        mc_override_ctrl_update(gc_state.modal.override);
    }
#endif
    // [10. Dwell ]:
    if (gc_block.non_modal_command == NonModal::Dwell) {
        mc_dwell(int32_t(gc_block.values.p * 1000.0f));
    }
    // [11. Set active plane ]:
    gc_state.modal.plane_select = gc_block.modal.plane_select;
    // [12. Set length units ]:
    gc_state.modal.units = gc_block.modal.units;
    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED
    // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // NOTE: Not needed since always disabled.
    // [14. Cutter length compensation ]: G43.1 and G49 supported. G43 NOT SUPPORTED.
    // NOTE: If G43 were supported, its operation wouldn't be any different from G43.1 in terms
    // of execution. The error-checking step would simply load the offset value into the correct
    // axis of the block XYZ value array.
    if (axis_command == AxisCommand::ToolLengthOffset) {  // Indicates a change.
        gc_state.modal.tool_length = gc_block.modal.tool_length;
        if (gc_state.modal.tool_length == ToolLengthOffset::Cancel) {  // G49
            gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] = 0.0;
        }
        // else G43.1
        if (gc_state.tool_length_offset != gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS]) {
            gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
            system_flag_wco_change();
        }
    }
    // [15. Coordinate system selection ]:
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
        gc_state.modal.coord_select = gc_block.modal.coord_select;
        memcpy(gc_state.coord_system, block_coord_system, sizeof(gc_state.coord_system));
        system_flag_wco_change();
    }
    // [16. Set path control mode ]: G61.1/G64 NOT SUPPORTED
    // gc_state.modal.control = gc_block.modal.control; // NOTE: Always default.
    // [17. Set distance mode ]:
    gc_state.modal.distance = gc_block.modal.distance;
    // [18. Set retract mode ]: NOT SUPPORTED
    // [19. Go to predefined position, Set G10, or Set axis offsets ]:
    switch (gc_block.non_modal_command) {
        case NonModal::SetCoordinateData:
            coords[coord_select]->set(coord_data);
            // 如果当前激活，则更新系统坐标系统。
            if (gc_state.modal.coord_select == coord_select) {
                memcpy(gc_state.coord_system, coord_data, sizeof(gc_state.coord_system));
                system_flag_wco_change();
            }
            break;
        case NonModal::GoHome0:
        case NonModal::GoHome1:
            // Move to intermediate position before going home. Obeys current coordinate system and offsets
            // and absolute and incremental modes.
            pl_data->motion.rapidMotion = 1;  // Set rapid motion flag.
            if (axis_command != AxisCommand::None) {
                limitsCheckSoft(gc_block.values.xyz); 
                // grbl_msg_sendf(CLIENT_SERIAL,
                //            MsgLevel::Warning,
                //            ">>1708 gc_block.values.xyz X:%f, Y:%f, Z:%f",
                //            gc_block.values.xyz[X_AXIS],
                //            gc_block.values.xyz[Y_AXIS],
                //            gc_block.values.xyz[Z_AXIS]);
                cartesian_to_motors(gc_block.values.xyz, pl_data, gc_state.position);
            }
            limitsCheckSoft(coord_data); 
            // grbl_msg_sendf(CLIENT_SERIAL,
            //                MsgLevel::Warning,
            //                ">>1717 coord_data X:%f, Y:%f, Z:%f",
            //                coord_data[X_AXIS],
            //                coord_data[Y_AXIS],
            //                coord_data[Z_AXIS]);
            cartesian_to_motors(coord_data, pl_data, gc_state.position);
            memcpy(gc_state.position, coord_data, sizeof(gc_state.position));
            break;
        case NonModal::SetHome0:
            coords[CoordIndex::G28]->set(gc_state.position);
            break;
        case NonModal::SetHome1:
            coords[CoordIndex::G30]->set(gc_state.position);
            break;
        case NonModal::SetCoordinateOffset:
            memcpy(gc_state.coord_offset, gc_block.values.xyz, sizeof(gc_block.values.xyz));
            system_flag_wco_change();
            break;
        case NonModal::ResetCoordinateOffset:
            clear_vector(gc_state.coord_offset);  // Disable G92 offsets by zeroing offset vector.
            system_flag_wco_change();
            break;
        case NonModal::SetMachinePosition:
            //memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
            //system_flag_wco_change();
            break;
        default:
            break;
    }
    // [20. Motion modes ]:
    // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes.
    // Enter motion modes only if there are axis words or a motion mode command word in the block.
    gc_state.modal.motion = gc_block.modal.motion;
    if (gc_state.modal.motion != Motion::None) {
        if (axis_command == AxisCommand::MotionMode) {
            GCUpdatePos gc_update_pos = GCUpdatePos::Target;
            if (gc_state.modal.motion == Motion::Linear) {
                limitsCheckSoft(gc_block.values.xyz);
                // grbl_msg_sendf(CLIENT_SERIAL,
                //            MsgLevel::Warning,
                //            ">>1756 gc_block.values.xyz X:%f, Y:%f, Z:%f",
                //            gc_block.values.xyz[X_AXIS],
                //            gc_block.values.xyz[Y_AXIS],
                //            gc_block.values.xyz[Z_AXIS]);
                cartesian_to_motors(gc_block.values.xyz, pl_data, gc_state.position);
            } else if (gc_state.modal.motion == Motion::Seek) {
                pl_data->motion.rapidMotion = 1;  // Set rapid motion flag.
                limitsCheckSoft(gc_block.values.xyz);
            // grbl_msg_sendf(CLIENT_SERIAL,
            //                MsgLevel::Warning,
            //                ">>1766 coord_data X:%f, Y:%f, Z:%f",
            //                coord_data[X_AXIS],
            //                coord_data[Y_AXIS],
            //                coord_data[Z_AXIS]);
                cartesian_to_motors(gc_block.values.xyz, pl_data, gc_state.position);
            } else if ((gc_state.modal.motion == Motion::CwArc) || (gc_state.modal.motion == Motion::CcwArc)) {
                mc_arc(gc_block.values.xyz,
                       pl_data,
                       gc_state.position,
                       gc_block.values.ijk,
                       gc_block.values.r,
                       axis_0,
                       axis_1,
                       axis_linear,
                       bit_istrue(gc_parser_flags, GCParserArcIsClockwise));
            } else {
                // NOTE: gc_block.values.xyz is returned from mc_probe_cycle with the updated position value. So
                // upon a successful probing cycle, the machine position and the returned value should be the same.
                // NOTE: gc_block.values.xyz 使用更新后的位置值从Mc_probe_cycle返回。所以
                //在一个成功的探测周期中，机器位置和返回值应该是相同的。
#ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
                pl_data->motion.noFeedOverride = 1;
#endif
                gc_update_pos = mc_probe_cycle(gc_block.values.xyz, pl_data, gc_parser_flags);
            }
            // As far as the parser is concerned, the position is now == target. In reality the
            // motion control system might still be processing the action and the real tool position
            // in any intermediate location.
            //就解析器而言，位置现在是== target。事实上

//运动控制系统可能仍在处理动作和真实的工具位置

//在任何中间位置。
            if (gc_update_pos == GCUpdatePos::Target) {
                memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));  // gc_state.position[] = gc_block.values.xyz[]
            } else if (gc_update_pos == GCUpdatePos::System) {
                gc_sync_position();  // gc_state.position[] = sys_position
            }                        // == GCUpdatePos::None
        }
    }
    // [21. Program flow ]:
    // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may
    // refill and can only be resumed by the cycle start run-time command.
    gc_state.modal.program_flow = gc_block.modal.program_flow;
    switch (gc_state.modal.program_flow) {
        case ProgramFlow::Running:
            break;
        case ProgramFlow::OptionalStop:
            // TODO - to support M1 we would need some code to determine whether to stop
            // Then either break or fall through to actually stop.
            break;
        case ProgramFlow::Paused:
            protocol_buffer_synchronize();  // Sync and finish all remaining buffered motions before moving on.
            if (sys.state != State::CheckMode) {
                sys_rt_exec_state.bit.feedHold = true;  // Use feed hold for program pause.
                protocol_execute_realtime();            // Execute suspend.
            }
            break;
        case ProgramFlow::CompletedM2:
        case ProgramFlow::CompletedM30:
            protocol_buffer_synchronize();  // Sync and finish all remaining buffered motions before moving on.
//当程序完成时，只有g代码的子集被重置为某些默认值，根据

// LinuxCNC的程序结束描述和测试。只有模态组[g代码1,2,3,5,7,12]

//和[M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]。剩下的模态组

// [g码4,6,8,10,13,14,15]和[m码4,5,6]以及模态词[F,S,T,H]不会重置。
            // Upon program complete, only a subset of g-codes reset to certain defaults, according to
            // LinuxCNC's program end descriptions and testing. Only modal groups [G-code 1,2,3,5,7,12]
            // and [M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]. The remaining modal groups
            // [G-code 4,6,8,10,13,14,15] and [M-code 4,5,6] and the modal words [F,S,T,H] do not reset.
            gc_state.modal.motion       = Motion::Linear;
            gc_state.modal.plane_select = Plane::XY;
            gc_state.modal.distance     = Distance::Absolute;
            gc_state.modal.feed_rate    = FeedRate::UnitsPerMin;
            // gc_state.modal.cutter_comp = CutterComp::Disable; // Not supported.
            gc_state.modal.coord_select = CoordIndex::G54;
            gc_state.modal.spindle      = SpindleState::Disable;
            gc_state.modal.coolant      = {};
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
#    ifdef DEACTIVATE_PARKING_UPON_INIT
            gc_state.modal.override = Override::Disabled;
#    else
            gc_state.modal.override = Override::ParkingMotion;
#    endif
#endif
            // gc_state.modal.override = OVERRIDE_DISABLE; // Not supported.
#ifdef RESTORE_OVERRIDES_AFTER_PROGRAM_END
            sys.f_override        = FeedOverride::Default;
            sys.r_override        = RapidOverride::Default;
            sys.spindle_speed_ovr = SpindleSpeedOverride::Default;
#endif
            // Execute coordinate change and spindle/coolant stop.
            if (sys.state != State::CheckMode) {
                coords[gc_state.modal.coord_select]->get(gc_state.coord_system);
                system_flag_wco_change();  // Set to refresh immediately just in case something altered.
                spindle->set_state(SpindleState::Disable, 0);
                coolant_off();
            }
            report_feedback_message(Message::ProgramEnd);
            user_m30();
            break;
    }
    gc_state.modal.program_flow = ProgramFlow::Running;  // Reset program flow.

    // TODO: % to denote start of program.
    return Error::Ok;
}

/*
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Override control (TBD)
  - Tool changes
  - Switches

   (*) Indicates optional parameter, enabled through config.h and re-compile
   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G81 - G89} (Motion modes: Canned cycles)
   group 4 = {M1} (Optional stop, ignored)
   group 6 = {M6} (Tool change)
   group 7 = {G41, G42} cutter radius compensation (G40 is supported)
   group 8 = {G43} tool length offset (G43.1/G49 are supported)
   group 8 = {M7*} enable mist coolant (* Compile-option)
   group 9 = {M48, M49} enable/disable feed and speed override switches
   group 10 = {G98, G99} return mode canned cycles
   group 13 = {G61.1, G64} path control mode (G61 is supported)
*/
