import 'dart:async';
import 'dart:convert';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'package:path_provider/path_provider.dart';
import 'package:record/record.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'dart:io';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp, DeviceOrientation.portraitDown]);
  runApp(const PepperMobileApp());
}

class PepperMobileApp extends StatelessWidget {
  const PepperMobileApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Pepper Controller & Chat',
      debugShowCheckedModeBanner: false,
      themeMode: ThemeMode.dark,
      darkTheme: ThemeData(
        brightness: Brightness.dark,
        scaffoldBackgroundColor: const Color(0xFF090D16),
        colorScheme: const ColorScheme.dark(primary: Color(0xFF00E5FF), secondary: Color(0xFF7C4DFF), surface: Color(0xFF131A29), error: Color(0xFFFF5252)),
        fontFamily: 'sans-serif',
      ),
      home: const MainNavigationScreen(),
    );
  }
}

class MainNavigationScreen extends StatefulWidget {
  const MainNavigationScreen({super.key});

  @override
  State<MainNavigationScreen> createState() => _MainNavigationScreenState();
}

class _MainNavigationScreenState extends State<MainNavigationScreen> {
  int _currentTabIndex = 0;

  // Connection config
  // Dos conexiones: en casa la IP del PC (_host:_port); fuera, la URL de ngrok (_remoteUrl).
  // Si la que se usa deja de responder, se prueba la otra (_useRemote).
  // Usuario/clave (ngrok --basic-auth) van en la URL; dart:io los manda como Basic auth.
  String _host = '10.0.2.2';
  int _port = 8080;
  String _remoteUrl = '';
  String _user = '';
  String _pass = '';
  bool _useRemote = false;
  int _failures = 0;
  bool _isConnected = false;

  // Telemetry
  double _fps = 0.0;
  String _lastSpeech = 'Esperando conexión...';
  double _robotX = 0.0;
  double _robotY = 0.0;
  double _robotYaw = 0.0;

  Timer? _telemetryTimer;

  bool get _remote => _useRemote && _remoteUrl.isNotEmpty;
  String get hostDisplayName => _remote ? Uri.parse(_remoteUrl).host : '$_host:$_port';
  String get _baseUrl {
    if (!_remote) return 'http://$_host:$_port';
    final base = Uri.parse(_remoteUrl.replaceAll(RegExp(r'/+$'), ''));
    return (_user.isEmpty ? base : base.replace(userInfo: '$_user:$_pass')).toString();
  }

  String get _streamUrl => '$_baseUrl/stream/';
  String get _statusUrl => '$_baseUrl/api/status';
  String get _cmdVelUrl => '$_baseUrl/api/cmd_vel';
  String get _headUrl => '$_baseUrl/api/head';
  String get _sayUrl => '$_baseUrl/api/say';
  String get _chatUrl => '$_baseUrl/api/chat';

  @override
  void initState() {
    super.initState();
    _loadSettings();
    _startTelemetryLoop();
  }

  Future<void> _loadSettings() async {
    final prefs = await SharedPreferences.getInstance();
    if (!mounted) return;
    setState(() {
      _host = prefs.getString('host') ?? _host;
      _port = prefs.getInt('port') ?? _port;
      _remoteUrl = prefs.getString('remote_url') ?? '';
      _user = prefs.getString('user') ?? '';
      _pass = prefs.getString('pass') ?? '';
    });
  }

  Future<void> _saveSettings() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('host', _host);
    await prefs.setInt('port', _port);
    await prefs.setString('remote_url', _remoteUrl);
    await prefs.setString('user', _user);
    await prefs.setString('pass', _pass);
  }

  @override
  void dispose() {
    _telemetryTimer?.cancel();
    super.dispose();
  }

  void _startTelemetryLoop() {
    _telemetryTimer?.cancel();
    _telemetryTimer = Timer.periodic(const Duration(milliseconds: 1000), (_) {
      _fetchStatus();
    });
  }

  Future<void> _fetchStatus() async {
    try {
      final res = await http.get(Uri.parse(_statusUrl)).timeout(const Duration(seconds: 3));
      if (res.statusCode == 200) {
        final data = json.decode(res.body);
        if (mounted) {
          setState(() {
            _isConnected = true;
            _failures = 0;
            _fps = (data['fps'] as num?)?.toDouble() ?? 0.0;
            _robotX = (data['robot_x'] as num?)?.toDouble() ?? 0.0;
            _robotY = (data['robot_y'] as num?)?.toDouble() ?? 0.0;
            _robotYaw = (data['robot_yaw'] as num?)?.toDouble() ?? 0.0;
            final speech = data['last_speech'] as String?;
            if (speech != null && speech.isNotEmpty) {
              _lastSpeech = speech;
            }
          });
        }
      } else {
        _onStatusFailure();
      }
    } catch (_) {
      _onStatusFailure();
    }
  }

  void _onStatusFailure() {
    if (!mounted) return;
    setState(() {
      _isConnected = false;
      // 2 fallos seguidos → probar la otra conexión (si hay URL de ngrok)
      if (++_failures >= 2 && _remoteUrl.isNotEmpty) {
        _useRemote = !_useRemote;
        _failures = 0;
      }
    });
  }

  void _openSettingsDialog() {
    final hostCtrl = TextEditingController(text: _host);
    final portCtrl = TextEditingController(text: _port.toString());
    final remoteCtrl = TextEditingController(text: _remoteUrl);
    final userCtrl = TextEditingController(text: _user);
    final passCtrl = TextEditingController(text: _pass);

    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF131A29),
        title: const Row(
          children: [
            Icon(Icons.settings, color: Color(0xFF00E5FF)),
            SizedBox(width: 8),
            Text('Configuración de Conexión', style: TextStyle(fontSize: 18)),
          ],
        ),
        content: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              TextField(
                controller: hostCtrl,
                decoration: const InputDecoration(
                  labelText: 'En casa: IP del PC',
                  helperText: 'Emulador: 10.0.2.2 | Teléfono: 192.168.0.9',
                  labelStyle: TextStyle(color: Color(0xFF00E5FF)),
                ),
              ),
              const SizedBox(height: 12),
              TextField(
                controller: portCtrl,
                keyboardType: TextInputType.number,
                decoration: const InputDecoration(
                  labelText: 'Puerto Bridge',
                  labelStyle: TextStyle(color: Color(0xFF00E5FF)),
                ),
              ),
              const SizedBox(height: 12),
              TextField(
                controller: remoteCtrl,
                keyboardType: TextInputType.url,
                decoration: const InputDecoration(
                  labelText: 'Fuera de casa: URL de ngrok (opcional)',
                  helperText: 'https://xxx.ngrok-free.app — se usa si la IP no responde',
                  helperMaxLines: 2,
                  labelStyle: TextStyle(color: Color(0xFF00E5FF)),
                ),
              ),
              const SizedBox(height: 12),
              TextField(
                controller: userCtrl,
                decoration: const InputDecoration(
                  labelText: 'Usuario (opcional)',
                  helperText: 'El de ngrok --basic-auth',
                  labelStyle: TextStyle(color: Color(0xFF00E5FF)),
                ),
              ),
              const SizedBox(height: 12),
              TextField(
                controller: passCtrl,
                obscureText: true,
                decoration: const InputDecoration(
                  labelText: 'Clave (opcional)',
                  labelStyle: TextStyle(color: Color(0xFF00E5FF)),
                ),
              ),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancelar', style: TextStyle(color: Colors.white70)),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFF00E5FF)),
            onPressed: () {
              setState(() {
                _host = hostCtrl.text.trim();
                _port = int.tryParse(portCtrl.text.trim()) ?? 8080;
                _remoteUrl = remoteCtrl.text.trim();
                _useRemote = false;
                _failures = 0;
                _user = userCtrl.text.trim();
                _pass = passCtrl.text.trim();
              });
              _saveSettings();
              Navigator.pop(ctx);
              _fetchStatus();
            },
            child: const Text(
              'Guardar',
              style: TextStyle(color: Colors.black, fontWeight: FontWeight.bold),
            ),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: const Color(0xFF0F1523),
        elevation: 0,
        title: Row(
          children: [
            Container(
              padding: const EdgeInsets.all(6),
              decoration: BoxDecoration(color: const Color(0xFF00E5FF).withOpacity(0.15), borderRadius: BorderRadius.circular(8)),
              child: const Icon(Icons.smart_toy_rounded, color: Color(0xFF00E5FF), size: 24),
            ),
            const SizedBox(width: 10),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('PEPPER ROBOT', style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold, letterSpacing: 1.2)),
                Text('ROS 2 Jazzy Simulación', style: TextStyle(fontSize: 11, color: Colors.white.withOpacity(0.6))),
              ],
            ),
          ],
        ),
        actions: [
          // Connection status badge
          Container(
            margin: const EdgeInsets.symmetric(vertical: 12),
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
            decoration: BoxDecoration(
              color: _isConnected ? const Color(0xFF00E676).withOpacity(0.15) : Colors.red.withOpacity(0.15),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: _isConnected ? const Color(0xFF00E676) : Colors.red, width: 1),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Container(
                  width: 8,
                  height: 8,
                  decoration: BoxDecoration(color: _isConnected ? const Color(0xFF00E676) : Colors.red, shape: BoxShape.circle),
                ),
                const SizedBox(width: 6),
                Text(
                  _isConnected ? (_remote ? 'EN LÍNEA · NGROK' : 'EN LÍNEA · CASA') : 'OFFLINE',
                  style: TextStyle(fontSize: 11, fontWeight: FontWeight.bold, color: _isConnected ? const Color(0xFF00E676) : Colors.red),
                ),
              ],
            ),
          ),
          IconButton(
            icon: const Icon(Icons.tune_rounded, color: Color(0xFF00E5FF)),
            onPressed: _openSettingsDialog,
          ),
          const SizedBox(width: 4),
        ],
      ),
      body: IndexedStack(
        index: _currentTabIndex,
        children: [
          // Tab 0: Camera & Controls
          DashboardView(
            host: hostDisplayName,
            port: _port,
            isConnected: _isConnected,
            fps: _fps,
            robotX: _robotX,
            robotY: _robotY,
            robotYaw: _robotYaw,
            lastSpeech: _lastSpeech,
            cmdVelUrl: _cmdVelUrl,
            headUrl: _headUrl,
            sayUrl: _sayUrl,
            streamBaseUrl: _streamUrl,
          ),

          // Tab 1: Interactive Chat with Pepper
          ChatView(
            chatUrl: _chatUrl,
            isConnected: _isConnected,
            onActionNotification: (msg) {
              _fetchStatus();
            },
          ),

          // Tab 2: un visitante del museo le habla a Pepper
          VisitorView(
            baseUrl: _baseUrl,
            isConnected: _isConnected,
            isActive: _currentTabIndex == 2,
          ),
        ],
      ),
      bottomNavigationBar: Container(
        decoration: BoxDecoration(
          color: const Color(0xFF0F1523),
          border: Border(top: BorderSide(color: Colors.white.withOpacity(0.08))),
        ),
        child: BottomNavigationBar(
          currentIndex: _currentTabIndex,
          onTap: (idx) => setState(() => _currentTabIndex = idx),
          backgroundColor: Colors.transparent,
          selectedItemColor: const Color(0xFF00E5FF),
          unselectedItemColor: Colors.white38,
          selectedLabelStyle: const TextStyle(fontWeight: FontWeight.bold, fontSize: 12),
          elevation: 0,
          items: const [
            BottomNavigationBarItem(
              icon: Icon(Icons.videocam_rounded),
              activeIcon: Icon(Icons.videocam_rounded, color: Color(0xFF00E5FF)),
              label: 'Cámara & Control',
            ),
            BottomNavigationBarItem(
              icon: Icon(Icons.chat_bubble_outline_rounded),
              activeIcon: Icon(Icons.chat_bubble_rounded, color: Color(0xFF00E5FF)),
              label: 'Chat Pepper',
            ),
            BottomNavigationBarItem(
              icon: Icon(Icons.person_pin_circle_outlined),
              activeIcon: Icon(Icons.person_pin_circle_rounded, color: Color(0xFF00E5FF)),
              label: 'Visitante',
            ),
          ],
        ),
      ),
    );
  }
}

// -----------------------------------------------------------------------------
// TAB 0: DASHBOARD VIEW
// -----------------------------------------------------------------------------
class DashboardView extends StatefulWidget {
  final String host;
  final int port;
  final bool isConnected;
  final double fps;
  final double robotX;
  final double robotY;
  final double robotYaw;
  final String lastSpeech;
  final String cmdVelUrl;
  final String headUrl;
  final String sayUrl;
  final String streamBaseUrl;

  const DashboardView({
    super.key,
    required this.host,
    required this.port,
    required this.isConnected,
    required this.fps,
    required this.robotX,
    required this.robotY,
    required this.robotYaw,
    required this.lastSpeech,
    required this.cmdVelUrl,
    required this.headUrl,
    required this.sayUrl,
    required this.streamBaseUrl,
  });

  @override
  State<DashboardView> createState() => _DashboardViewState();
}

class _DashboardViewState extends State<DashboardView> {
  // Vistas del bridge (/stream/<nombre>): frontal, frontal con las cajas de YOLO, inferior
  String _activeCamera = 'front';
  static const _cameraOrder = ['front', 'yolo', 'bottom'];
  static const _cameraNames = {'front': 'FRONTAL', 'yolo': 'FRONTAL + YOLO', 'bottom': 'INFERIOR'};
  String get _nextCamera => _cameraOrder[(_cameraOrder.indexOf(_activeCamera) + 1) % _cameraOrder.length];
  bool _showOverlay = true;

  double _headYaw = 0.0;
  double _headPitch = 0.0;

  final double _maxLinearSpeed = 0.25;
  final double _maxAngularSpeed = 0.55;

  final TextEditingController _speechInputCtrl = TextEditingController();

  @override
  void dispose() {
    _speechInputCtrl.dispose();
    super.dispose();
  }

  Future<void> _sendCmdVel(double linearX, double angularZ) async {
    if (!widget.isConnected) return;
    try {
      await http.post(Uri.parse(widget.cmdVelUrl), headers: {'Content-Type': 'application/json'}, body: json.encode({'linear_x': linearX, 'angular_z': angularZ}));
    } catch (_) {}
  }

  Future<void> _sendHead(double yaw, double pitch) async {
    if (!widget.isConnected) return;
    try {
      await http.post(Uri.parse(widget.headUrl), headers: {'Content-Type': 'application/json'}, body: json.encode({'yaw': yaw, 'pitch': pitch}));
    } catch (_) {}
  }

  Future<void> _sendSay(String text) async {
    if (text.trim().isEmpty) return;
    try {
      await http.post(Uri.parse(widget.sayUrl), headers: {'Content-Type': 'application/json'}, body: json.encode({'text': text.trim()}));
      _speechInputCtrl.clear();
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('Pepper dirá: "$text"'), backgroundColor: const Color(0xFF00E5FF), duration: const Duration(seconds: 2)));
    } catch (_) {}
  }

  void _emergencyStop() {
    _sendCmdVel(0.0, 0.0);
    HapticFeedback.heavyImpact();
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('🛑 PARADA DE EMERGENCIA ENVIADA'), backgroundColor: Colors.redAccent, duration: Duration(seconds: 1)));
  }

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(12.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          _buildCameraCard(),
          const SizedBox(height: 12),
          _buildTelemetryBanner(),
          const SizedBox(height: 12),
          _buildControlPanel(),
          const SizedBox(height: 12),
          _buildHeadControlCard(),
          const SizedBox(height: 12),
          _buildSpeechCard(),
          const SizedBox(height: 24),
        ],
      ),
    );
  }

  Widget _buildCameraCard() {
    final streamUrl = '${widget.streamBaseUrl}$_activeCamera';

    return Container(
      decoration: BoxDecoration(
        color: const Color(0xFF131A29),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: const Color(0xFF00E5FF).withOpacity(0.2), width: 1.5),
        boxShadow: [BoxShadow(color: Colors.black.withOpacity(0.4), blurRadius: 10, offset: const Offset(0, 4))],
      ),
      clipBehavior: Clip.antiAlias,
      child: Column(
        children: [
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
            color: const Color(0xFF1B2438),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Row(
                  children: [
                    const Icon(Icons.videocam_rounded, color: Color(0xFF00E5FF), size: 18),
                    const SizedBox(width: 6),
                    Text('CÁMARA ${_cameraNames[_activeCamera]}', style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 13, letterSpacing: 0.8)),
                  ],
                ),
                Row(
                  children: [
                    InkWell(
                      onTap: () {
                        setState(() {
                          _activeCamera = _nextCamera;
                        });
                      },
                      borderRadius: BorderRadius.circular(8),
                      child: Container(
                        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                        decoration: BoxDecoration(color: const Color(0xFF00E5FF).withOpacity(0.15), borderRadius: BorderRadius.circular(8)),
                        child: Row(
                          children: [
                            const Icon(Icons.flip_camera_ios_rounded, size: 14, color: Color(0xFF00E5FF)),
                            const SizedBox(width: 4),
                            Text(
                              _nextCamera == 'yolo' ? 'YOLO' : (_nextCamera == 'front' ? 'Frontal' : 'Inferior'),
                              style: const TextStyle(fontSize: 11, color: Color(0xFF00E5FF), fontWeight: FontWeight.w600),
                            ),
                          ],
                        ),
                      ),
                    ),
                    const SizedBox(width: 8),
                    IconButton(
                      icon: Icon(_showOverlay ? Icons.grid_on_rounded : Icons.grid_off_rounded, color: _showOverlay ? const Color(0xFF00E5FF) : Colors.white38, size: 18),
                      padding: EdgeInsets.zero,
                      constraints: const BoxConstraints(),
                      onPressed: () => setState(() => _showOverlay = !_showOverlay),
                    ),
                  ],
                ),
              ],
            ),
          ),
          AspectRatio(
            aspectRatio: 640 / 480,
            child: Stack(
              fit: StackFit.expand,
              children: [
                widget.isConnected
                    ? Mjpeg(
                        isLive: true,
                        stream: streamUrl,
                        timeout: const Duration(seconds: 3),
                        error: (context, error, stack) => _buildErrorCameraView('Reconectando transmisión...'),
                        loading: (context) => const Center(child: CircularProgressIndicator(color: Color(0xFF00E5FF))),
                      )
                    : _buildErrorCameraView('Sin conexión con ${widget.host}'),
                if (_showOverlay) _buildHUDOverlay(),
                Positioned(
                  top: 10,
                  left: 10,
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                    decoration: BoxDecoration(
                      color: Colors.black.withOpacity(0.7),
                      borderRadius: BorderRadius.circular(6),
                      border: Border.all(color: Colors.white12),
                    ),
                    child: Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Container(
                          width: 8,
                          height: 8,
                          decoration: const BoxDecoration(color: Colors.redAccent, shape: BoxShape.circle),
                        ),
                        const SizedBox(width: 6),
                        const Text(
                          'REC',
                          style: TextStyle(fontSize: 10, fontWeight: FontWeight.bold, color: Colors.white),
                        ),
                        const SizedBox(width: 8),
                        Text(
                          '${widget.fps.toStringAsFixed(1)} FPS',
                          style: const TextStyle(fontSize: 10, color: Color(0xFF00E5FF), fontWeight: FontWeight.bold),
                        ),
                      ],
                    ),
                  ),
                ),
                Positioned(
                  bottom: 10,
                  right: 10,
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                    decoration: BoxDecoration(color: Colors.black.withOpacity(0.7), borderRadius: BorderRadius.circular(4)),
                    child: const Text(
                      '640 × 480',
                      style: TextStyle(fontSize: 10, color: Colors.white70, fontFamily: 'monospace'),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildErrorCameraView(String message) {
    return Container(
      color: const Color(0xFF0A0E17),
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Icon(Icons.signal_wifi_connected_no_internet_4_rounded, size: 48, color: Colors.white24),
            const SizedBox(height: 10),
            Text(message, style: const TextStyle(color: Colors.white54, fontSize: 13)),
          ],
        ),
      ),
    );
  }

  Widget _buildHUDOverlay() {
    return IgnorePointer(
      child: Center(
        child: SizedBox(width: 80, height: 80, child: CustomPaint(painter: CrosshairPainter())),
      ),
    );
  }

  Widget _buildTelemetryBanner() {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: const Color(0xFF131A29),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.white10),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              _buildTelemetryItem(Icons.near_me_rounded, 'Pose X', '${widget.robotX.toStringAsFixed(2)} m'),
              _buildTelemetryItem(Icons.near_me_rounded, 'Pose Y', '${widget.robotY.toStringAsFixed(2)} m'),
              _buildTelemetryItem(Icons.rotate_right_rounded, 'Yaw', '${(widget.robotYaw * 180 / math.pi).toStringAsFixed(1)}°'),
            ],
          ),
          const Divider(height: 16, color: Colors.white10),
          Row(
            children: [
              const Icon(Icons.record_voice_over_rounded, size: 16, color: Color(0xFF7C4DFF)),
              const SizedBox(width: 8),
              const Text(
                'Pepper dice: ',
                style: TextStyle(fontSize: 12, color: Colors.white70, fontWeight: FontWeight.bold),
              ),
              Expanded(
                child: Text(
                  widget.lastSpeech.isEmpty ? '(en silencio)' : '"${widget.lastSpeech}"',
                  style: const TextStyle(fontSize: 12, color: Color(0xFF00E5FF), fontStyle: FontStyle.italic),
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildTelemetryItem(IconData icon, String label, String val) {
    return Row(
      children: [
        Icon(icon, size: 16, color: const Color(0xFF00E5FF)),
        const SizedBox(width: 6),
        Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(label, style: const TextStyle(fontSize: 10, color: Colors.white54)),
            Text(
              val,
              style: const TextStyle(fontSize: 13, fontWeight: FontWeight.bold, color: Colors.white),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildControlPanel() {
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: const Color(0xFF131A29),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.white10),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              const Row(
                children: [
                  Icon(Icons.sports_esports_rounded, color: Color(0xFF00E5FF), size: 20),
                  SizedBox(width: 8),
                  Text('Control de Movimiento', style: TextStyle(fontSize: 15, fontWeight: FontWeight.bold)),
                ],
              ),
              ElevatedButton.icon(
                onPressed: _emergencyStop,
                icon: const Icon(Icons.dangerous_rounded, size: 16, color: Colors.white),
                label: const Text('PARADA', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 12)),
                style: ElevatedButton.styleFrom(
                  backgroundColor: const Color(0xFFFF1744),
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                ),
              ),
            ],
          ),
          const SizedBox(height: 14),
          Center(
            child: SizedBox(
              width: 220,
              height: 220,
              child: Stack(
                alignment: Alignment.center,
                children: [
                  GestureDetector(
                    onTap: () => _sendCmdVel(0.0, 0.0),
                    child: Container(
                      width: 60,
                      height: 60,
                      decoration: BoxDecoration(
                        color: const Color(0xFF1F2B42),
                        shape: BoxShape.circle,
                        border: Border.all(color: const Color(0xFF00E5FF).withOpacity(0.5), width: 2),
                      ),
                      child: const Center(
                        child: Text(
                          'STOP',
                          style: TextStyle(fontSize: 11, fontWeight: FontWeight.bold, color: Color(0xFF00E5FF)),
                        ),
                      ),
                    ),
                  ),
                  Positioned(top: 0, child: _buildDPadButton(Icons.arrow_upward_rounded, () => _sendCmdVel(_maxLinearSpeed, 0.0), () => _sendCmdVel(0.0, 0.0))),
                  Positioned(bottom: 0, child: _buildDPadButton(Icons.arrow_downward_rounded, () => _sendCmdVel(-_maxLinearSpeed, 0.0), () => _sendCmdVel(0.0, 0.0))),
                  Positioned(left: 0, child: _buildDPadButton(Icons.arrow_back_rounded, () => _sendCmdVel(0.0, _maxAngularSpeed), () => _sendCmdVel(0.0, 0.0))),
                  Positioned(right: 0, child: _buildDPadButton(Icons.arrow_forward_rounded, () => _sendCmdVel(0.0, -_maxAngularSpeed), () => _sendCmdVel(0.0, 0.0))),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDPadButton(IconData icon, VoidCallback onPressStart, VoidCallback onPressEnd) {
    return GestureDetector(
      onTapDown: (_) {
        HapticFeedback.lightImpact();
        onPressStart();
      },
      onTapUp: (_) => onPressEnd(),
      onTapCancel: () => onPressEnd(),
      child: Container(
        width: 58,
        height: 58,
        decoration: BoxDecoration(
          color: const Color(0xFF1E2738),
          borderRadius: BorderRadius.circular(14),
          border: Border.all(color: const Color(0xFF00E5FF).withOpacity(0.3), width: 1.5),
          boxShadow: [BoxShadow(color: Colors.black.withOpacity(0.3), blurRadius: 6, offset: const Offset(0, 2))],
        ),
        child: Icon(icon, color: const Color(0xFF00E5FF), size: 30),
      ),
    );
  }

  Widget _buildHeadControlCard() {
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: const Color(0xFF131A29),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.white10),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              const Row(
                children: [
                  Icon(Icons.face_rounded, color: Color(0xFF7C4DFF), size: 20),
                  SizedBox(width: 8),
                  Text('Control de Cabeza', style: TextStyle(fontSize: 15, fontWeight: FontWeight.bold)),
                ],
              ),
              TextButton(
                onPressed: () {
                  setState(() {
                    _headYaw = 0.0;
                    _headPitch = 0.0;
                  });
                  _sendHead(0.0, 0.0);
                },
                child: const Text('Centrar', style: TextStyle(color: Color(0xFF00E5FF), fontSize: 12)),
              ),
            ],
          ),
          const SizedBox(height: 8),
          Row(
            children: [
              const SizedBox(
                width: 70,
                child: Text('Yaw (Giro):', style: TextStyle(fontSize: 12, color: Colors.white70)),
              ),
              Expanded(
                child: Slider(
                  value: _headYaw,
                  min: -1.5,
                  max: 1.5,
                  activeColor: const Color(0xFF00E5FF),
                  inactiveColor: Colors.white10,
                  onChanged: (val) {
                    setState(() => _headYaw = val);
                    _sendHead(_headYaw, _headPitch);
                  },
                ),
              ),
              SizedBox(
                width: 45,
                child: Text('${(_headYaw * 180 / math.pi).toStringAsFixed(0)}°', style: const TextStyle(fontSize: 12, color: Colors.white70)),
              ),
            ],
          ),
          Row(
            children: [
              const SizedBox(
                width: 70,
                child: Text('Pitch (Inclin):', style: TextStyle(fontSize: 12, color: Colors.white70)),
              ),
              Expanded(
                child: Slider(
                  value: _headPitch,
                  min: -0.5,
                  max: 0.5,
                  activeColor: const Color(0xFF7C4DFF),
                  inactiveColor: Colors.white10,
                  onChanged: (val) {
                    setState(() => _headPitch = val);
                    _sendHead(_headYaw, _headPitch);
                  },
                ),
              ),
              SizedBox(
                width: 45,
                child: Text('${(_headPitch * 180 / math.pi).toStringAsFixed(0)}°', style: const TextStyle(fontSize: 12, color: Colors.white70)),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildSpeechCard() {
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: const Color(0xFF131A29),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.white10),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Row(
            children: [
              Icon(Icons.campaign_rounded, color: Color(0xFF00E5FF), size: 20),
              SizedBox(width: 8),
              Text('Hacer Hablar a Pepper (/pepper/say)', style: TextStyle(fontSize: 15, fontWeight: FontWeight.bold)),
            ],
          ),
          const SizedBox(height: 10),
          Row(
            children: [
              Expanded(
                child: TextField(
                  controller: _speechInputCtrl,
                  decoration: InputDecoration(
                    hintText: 'Ej: Hola visitante, bienvenido...',
                    hintStyle: const TextStyle(color: Colors.white30, fontSize: 13),
                    filled: true,
                    fillColor: const Color(0xFF0F1523),
                    contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
                    border: OutlineInputBorder(borderRadius: BorderRadius.circular(10), borderSide: BorderSide.none),
                  ),
                  onSubmitted: _sendSay,
                ),
              ),
              const SizedBox(width: 8),
              ElevatedButton(
                onPressed: () => _sendSay(_speechInputCtrl.text),
                style: ElevatedButton.styleFrom(
                  backgroundColor: const Color(0xFF00E5FF),
                  foregroundColor: Colors.black,
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
                  padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 12),
                ),
                child: const Icon(Icons.send_rounded, size: 18),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

// -----------------------------------------------------------------------------
// TAB 1: INTERACTIVE CHAT VIEW
// -----------------------------------------------------------------------------
class ChatMessageItem {
  final String sender; // 'user' or 'pepper'
  final String text;
  final DateTime time;

  ChatMessageItem({required this.sender, required this.text, required this.time});
}

class ChatView extends StatefulWidget {
  final String chatUrl;
  final bool isConnected;
  final ValueChanged<String>? onActionNotification;

  const ChatView({super.key, required this.chatUrl, required this.isConnected, this.onActionNotification});

  @override
  State<ChatView> createState() => _ChatViewState();
}

class _ChatViewState extends State<ChatView> {
  final List<ChatMessageItem> _messages = [];
  final TextEditingController _msgInputCtrl = TextEditingController();
  final ScrollController _scrollCtrl = ScrollController();
  bool _isSending = false;

  // Voz: se graba un WAV en el teléfono y el bridge lo transcribe (Whisper de voice_listener)
  final AudioRecorder _recorder = AudioRecorder();
  bool _isRecording = false;
  bool _isTranscribing = false;
  Timer? _recordLimit;
  static const _maxRecording = Duration(seconds: 10);

  String get _voiceUrl => widget.chatUrl.replaceFirst('/api/chat', '/api/voice');

  final List<String> _quickCommands = [
    '👋 ¡Hola Pepper!',
    '📍 ¿Dónde estás?',
    '👀 ¿Qué ves?',
    '⬆️ Avanza un paso',
    '⬇️ Retrocede un paso',
    '🔄 Gira a la izquierda',
    '🔄 Gira a la derecha',
    '🛑 ¡Detente!',
    '🤖 ¿Quién eres?',
    '🔍 Buscar visitantes',
  ];

  @override
  void initState() {
    super.initState();
    _loadInitialHistory();
  }

  @override
  void dispose() {
    _recordLimit?.cancel();
    _recorder.dispose();
    _msgInputCtrl.dispose();
    _scrollCtrl.dispose();
    super.dispose();
  }

  Future<void> _toggleRecording() async {
    if (_isTranscribing || _isSending) return;
    if (_isRecording) {
      await _stopAndTranscribe();
      return;
    }
    final allowed = await _recorder.hasPermission();
    if (!mounted) return;
    if (!allowed) {
      _showSnack('🎤 Sin permiso de micrófono');
      return;
    }
    final dir = await getTemporaryDirectory();
    await _recorder.start(const RecordConfig(encoder: AudioEncoder.wav, sampleRate: 16000, numChannels: 1), path: '${dir.path}/pepper_voz.wav');
    setState(() => _isRecording = true);
    _recordLimit = Timer(_maxRecording, _stopAndTranscribe);
  }

  Future<void> _stopAndTranscribe() async {
    _recordLimit?.cancel();
    if (!_isRecording) return;
    final path = await _recorder.stop();
    setState(() {
      _isRecording = false;
      _isTranscribing = true;
    });
    String? text;
    String? error;
    try {
      final bytes = await File(path!).readAsBytes();
      final res = await http.post(Uri.parse(_voiceUrl), headers: {'Content-Type': 'audio/wav'}, body: bytes).timeout(const Duration(seconds: 25));
      final data = json.decode(res.body);
      if (res.statusCode == 200) {
        text = (data['transcript'] as String? ?? '').trim();
      } else {
        error = data['error'] as String? ?? 'Error del servidor (${res.statusCode})';
      }
    } catch (e) {
      error = 'Sin conexión con el robot';
    }
    if (!mounted) return;
    setState(() => _isTranscribing = false);
    if (error != null) {
      _showSnack('🎤 $error');
    } else if (text == null || text.isEmpty) {
      _showSnack('🎤 No te entendí, inténtalo de nuevo');
    } else {
      _sendMessage(text);
    }
  }

  void _showSnack(String text) {
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(text), duration: const Duration(seconds: 3), backgroundColor: const Color(0xFF7C4DFF)));
  }

  Future<void> _loadInitialHistory() async {
    try {
      final res = await http.get(Uri.parse(widget.chatUrl)).timeout(const Duration(seconds: 2));
      if (res.statusCode == 200) {
        final data = json.decode(res.body);
        final list = data['history'] as List?;
        if (list != null && list.isNotEmpty) {
          setState(() {
            _messages.clear();
            for (var item in list) {
              _messages.add(
                ChatMessageItem(sender: item['sender'] ?? 'pepper', text: item['text'] ?? '', time: DateTime.fromMillisecondsSinceEpoch(((item['time'] as num?)?.toDouble() ?? 0).toInt() * 1000)),
              );
            }
          });
          _scrollToBottom();
          return;
        }
      }
    } catch (_) {}

    // Fallback default greeting if history endpoint is not reachable yet
    if (_messages.isEmpty) {
      setState(() {
        _messages.add(ChatMessageItem(sender: 'pepper', text: '¡Hola! Soy Pepper, tu robot humanoide y guía del museo. Escríbeme cualquier indicación o pregunta.', time: DateTime.now()));
      });
    }
  }

  void _scrollToBottom() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_scrollCtrl.hasClients) {
        _scrollCtrl.animateTo(_scrollCtrl.position.maxScrollExtent, duration: const Duration(milliseconds: 250), curve: Curves.easeOut);
      }
    });
  }

  Future<void> _sendMessage(String text) async {
    final cleanText = text.trim();
    if (cleanText.isEmpty || _isSending) return;

    _msgInputCtrl.clear();

    setState(() {
      _messages.add(ChatMessageItem(sender: 'user', text: cleanText, time: DateTime.now()));
      _isSending = true;
    });
    _scrollToBottom();

    try {
      final res = await http.post(Uri.parse(widget.chatUrl), headers: {'Content-Type': 'application/json'}, body: json.encode({'message': cleanText})).timeout(const Duration(seconds: 25));

      if (res.statusCode == 200) {
        final data = json.decode(res.body);
        final reply = data['reply'] as String? ?? 'Indicación recibida.';

        if (mounted) {
          setState(() {
            _messages.add(ChatMessageItem(sender: 'pepper', text: reply, time: DateTime.now()));
            _isSending = false;
          });
          _scrollToBottom();
          widget.onActionNotification?.call(reply);
        }
      } else {
        _handleSendError('Error del servidor (${res.statusCode})');
      }
    } catch (e) {
      _handleSendError('Sin conexión con el robot');
    }
  }

  void _handleSendError(String errorMsg) {
    if (mounted) {
      setState(() {
        _messages.add(ChatMessageItem(sender: 'pepper', text: '⚠️ $errorMsg. Revisa que el bridge esté activo.', time: DateTime.now()));
        _isSending = false;
      });
      _scrollToBottom();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        // Chat Header Banner
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
          color: const Color(0xFF131A29),
          child: Row(
            children: [
              Container(
                width: 36,
                height: 36,
                decoration: BoxDecoration(
                  gradient: const LinearGradient(colors: [Color(0xFF00E5FF), Color(0xFF7C4DFF)]),
                  shape: BoxShape.circle,
                  boxShadow: [BoxShadow(color: const Color(0xFF00E5FF).withOpacity(0.4), blurRadius: 8)],
                ),
                child: const Icon(Icons.smart_toy_rounded, color: Colors.black, size: 20),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text('Pepper Asistente & Guía', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 14)),
                    Text(widget.isConnected ? '🟢 En línea y escuchando' : '🔴 Desconectado', style: TextStyle(fontSize: 11, color: widget.isConnected ? const Color(0xFF00E676) : Colors.redAccent)),
                  ],
                ),
              ),
              IconButton(
                icon: const Icon(Icons.refresh_rounded, color: Colors.white54, size: 20),
                onPressed: _loadInitialHistory,
                tooltip: 'Recargar historial',
              ),
            ],
          ),
        ),

        // Message List
        Expanded(
          child: ListView.builder(
            controller: _scrollCtrl,
            padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 12),
            itemCount: _messages.length,
            itemBuilder: (context, index) {
              final msg = _messages[index];
              final isUser = msg.sender == 'user';
              return _buildMessageBubble(msg, isUser);
            },
          ),
        ),

        // Typing indicator
        if (_isSending)
          Padding(
            padding: const EdgeInsets.only(left: 16, bottom: 8),
            child: Row(
              children: [
                const SizedBox(width: 14, height: 14, child: CircularProgressIndicator(strokeWidth: 2, color: Color(0xFF00E5FF))),
                const SizedBox(width: 8),
                Text(
                  'Pepper está respondiendo...',
                  style: TextStyle(fontSize: 12, color: Colors.white.withOpacity(0.6), fontStyle: FontStyle.italic),
                ),
              ],
            ),
          ),

        // Quick Suggestion Chips
        Container(
          height: 40,
          margin: const EdgeInsets.only(bottom: 6),
          child: ListView.separated(
            scrollDirection: Axis.horizontal,
            padding: const EdgeInsets.symmetric(horizontal: 12),
            itemCount: _quickCommands.length,
            separatorBuilder: (_, __) => const SizedBox(width: 8),
            itemBuilder: (context, index) {
              final cmd = _quickCommands[index];
              return ActionChip(
                backgroundColor: const Color(0xFF162033),
                side: BorderSide(color: const Color(0xFF00E5FF).withOpacity(0.3)),
                label: Text(cmd, style: const TextStyle(fontSize: 11, color: Colors.white)),
                onPressed: () => _sendMessage(cmd),
              );
            },
          ),
        ),

        // Bottom Input Field
        Container(
          padding: const EdgeInsets.fromLTRB(10, 8, 10, 12),
          decoration: BoxDecoration(
            color: const Color(0xFF131A29),
            border: Border(top: BorderSide(color: Colors.white.withOpacity(0.08))),
          ),
          child: SafeArea(
            top: false,
            child: Row(
              children: [
                // Micrófono: tocar para empezar a hablar y otra vez para enviar
                _isTranscribing
                    ? const Padding(
                        padding: EdgeInsets.all(12),
                        child: SizedBox(width: 24, height: 24, child: CircularProgressIndicator(strokeWidth: 2, color: Color(0xFF00E5FF))),
                      )
                    : IconButton(
                        icon: Icon(_isRecording ? Icons.stop_circle_rounded : Icons.mic_rounded, color: _isRecording ? Colors.redAccent : Colors.white70),
                        tooltip: _isRecording ? 'Toca para enviar' : 'Hablar con Pepper',
                        onPressed: _toggleRecording,
                      ),
                Expanded(
                  child: TextField(
                    controller: _msgInputCtrl,
                    style: const TextStyle(fontSize: 14),
                    decoration: InputDecoration(
                      hintText: _isRecording
                          ? '🔴 Escuchando... toca ⏹ para enviar'
                          : _isTranscribing
                          ? 'Transcribiendo...'
                          : 'Habla o dale una orden a Pepper...',
                      hintStyle: const TextStyle(color: Colors.white30, fontSize: 13),
                      filled: true,
                      fillColor: const Color(0xFF0A0E17),
                      contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
                      border: OutlineInputBorder(borderRadius: BorderRadius.circular(24), borderSide: BorderSide.none),
                    ),
                    onSubmitted: _sendMessage,
                  ),
                ),
                const SizedBox(width: 8),
                Container(
                  decoration: const BoxDecoration(color: Color(0xFF00E5FF), shape: BoxShape.circle),
                  child: IconButton(
                    icon: const Icon(Icons.send_rounded, color: Colors.black, size: 20),
                    onPressed: () => _sendMessage(_msgInputCtrl.text),
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildMessageBubble(ChatMessageItem msg, bool isUser) {
    final timeStr = '${msg.time.hour.toString().padLeft(2, '0')}:${msg.time.minute.toString().padLeft(2, '0')}';

    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: isUser ? MainAxisAlignment.end : MainAxisAlignment.start,
        crossAxisAlignment: CrossAxisAlignment.end,
        children: [
          if (!isUser) ...[
            Container(
              width: 28,
              height: 28,
              margin: const EdgeInsets.only(right: 8, bottom: 2),
              decoration: BoxDecoration(
                color: const Color(0xFF1E283D),
                shape: BoxShape.circle,
                border: Border.all(color: const Color(0xFF00E5FF).withOpacity(0.5)),
              ),
              child: const Icon(Icons.smart_toy_rounded, color: Color(0xFF00E5FF), size: 16),
            ),
          ],
          Flexible(
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
              decoration: BoxDecoration(
                color: isUser ? const Color(0xFF00758A) : const Color(0xFF1B2438),
                borderRadius: BorderRadius.only(
                  topLeft: const Radius.circular(16),
                  topRight: const Radius.circular(16),
                  bottomLeft: Radius.circular(isUser ? 16 : 4),
                  bottomRight: Radius.circular(isUser ? 4 : 16),
                ),
                border: Border.all(color: isUser ? const Color(0xFF00E5FF).withOpacity(0.4) : Colors.white10, width: 1),
                boxShadow: [BoxShadow(color: Colors.black.withOpacity(0.2), blurRadius: 4, offset: const Offset(0, 2))],
              ),
              child: Column(
                crossAxisAlignment: isUser ? CrossAxisAlignment.end : CrossAxisAlignment.start,
                children: [
                  Text(msg.text, style: const TextStyle(fontSize: 13.5, color: Colors.white, height: 1.3)),
                  const SizedBox(height: 4),
                  Text(timeStr, style: TextStyle(fontSize: 10, color: isUser ? Colors.white60 : Colors.white38)),
                ],
              ),
            ),
          ),
          if (isUser) ...[const SizedBox(width: 4)],
        ],
      ),
    );
  }
}

class CrosshairPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = const Color(0xFF00E5FF).withOpacity(0.5)
      ..strokeWidth = 1.2
      ..style = PaintingStyle.stroke;

    final center = Offset(size.width / 2, size.height / 2);
    canvas.drawCircle(center, 18, paint);
    canvas.drawCircle(center, 2, paint..style = PaintingStyle.fill);

    paint.style = PaintingStyle.stroke;
    canvas.drawLine(Offset(center.dx - 28, center.dy), Offset(center.dx - 18, center.dy), paint);
    canvas.drawLine(Offset(center.dx + 18, center.dy), Offset(center.dx + 28, center.dy), paint);
    canvas.drawLine(Offset(center.dx, center.dy - 28), Offset(center.dx, center.dy - 18), paint);
    canvas.drawLine(Offset(center.dx, center.dy + 18), Offset(center.dx, center.dy + 28), paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

// -----------------------------------------------------------------------------
// TAB 2: VISITANTE — la voz es la de "alguien del museo": la atiende visitor_approach
// (busca a la persona quieta más cercana, se acerca a ~1 m y queda de frente). Al llegar empieza
// la charla: Gemini responde como guía hasta que el visitante se despide.
// -----------------------------------------------------------------------------
class VisitorView extends StatefulWidget {
  final String baseUrl;
  final bool isConnected;
  final bool isActive;

  const VisitorView({super.key, required this.baseUrl, required this.isConnected, required this.isActive});

  @override
  State<VisitorView> createState() => _VisitorViewState();
}

class _VisitorViewState extends State<VisitorView> {
  static const _accent = Color(0xFFFFB74D);
  static const _maxRecording = Duration(seconds: 10);
  // Pasos visibles del comportamiento (fases de /pepper/visitor_status)
  static const _steps = [
    ('Te escucho', Icons.hearing_rounded),
    ('Te busco', Icons.travel_explore_rounded),
    ('Me acerco', Icons.directions_walk_rounded),
    ('De frente', Icons.accessibility_new_rounded),
    ('Charlamos', Icons.forum_rounded),
  ];
  static const _stepOf = {'searching': 1, 'approaching': 2, 'facing': 3, 'talking': 4, 'ended': 5};
  static const _failures = {'nobody', 'nav_failed', 'error'};

  final AudioRecorder _recorder = AudioRecorder();
  final ScrollController _scrollCtrl = ScrollController();
  bool _isRecording = false;
  bool _isTranscribing = false;
  Timer? _recordLimit;
  Timer? _poll;

  String _phase = 'idle';
  String _statusText = '';
  List<dynamic> _log = [];

  @override
  void initState() {
    super.initState();
    _poll = Timer.periodic(const Duration(milliseconds: 800), (_) {
      if (widget.isActive) _fetchState();
    });
  }

  @override
  void dispose() {
    _poll?.cancel();
    _recordLimit?.cancel();
    _recorder.dispose();
    _scrollCtrl.dispose();
    super.dispose();
  }

  bool get _busy => const {'searching', 'approaching', 'facing'}.contains(_phase);
  bool get _talking => _phase == 'talking';

  Future<void> _endTalk() async {
    try {
      await http.post(Uri.parse('${widget.baseUrl}/api/visitor_end')).timeout(const Duration(seconds: 3));
    } catch (_) {}
    _fetchState();
  }

  Future<void> _fetchState() async {
    try {
      final res = await http.get(Uri.parse('${widget.baseUrl}/api/visitor')).timeout(const Duration(seconds: 2));
      if (res.statusCode != 200 || !mounted) return;
      final data = json.decode(res.body);
      final log = data['log'] as List? ?? [];
      final grew = log.length != _log.length;
      setState(() {
        _phase = data['phase'] as String? ?? 'idle';
        _statusText = (data['status'] as Map?)?['text'] as String? ?? '';
        _log = log;
      });
      if (grew) _scrollToBottom();
    } catch (_) {}
  }

  void _scrollToBottom() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_scrollCtrl.hasClients) {
        _scrollCtrl.animateTo(_scrollCtrl.position.maxScrollExtent, duration: const Duration(milliseconds: 250), curve: Curves.easeOut);
      }
    });
  }

  Future<void> _toggleRecording() async {
    if (_isTranscribing) return;
    if (_isRecording) {
      await _stopAndSend();
      return;
    }
    if (_busy) {
      _showSnack('Pepper todavía está atendiendo la llamada anterior');
      return;
    }
    final allowed = await _recorder.hasPermission();
    if (!mounted) return;
    if (!allowed) {
      _showSnack('🎤 Sin permiso de micrófono');
      return;
    }
    final dir = await getTemporaryDirectory();
    await _recorder.start(const RecordConfig(encoder: AudioEncoder.wav, sampleRate: 16000, numChannels: 1), path: '${dir.path}/pepper_visitante.wav');
    setState(() => _isRecording = true);
    _recordLimit = Timer(_maxRecording, _stopAndSend);
  }

  Future<void> _stopAndSend() async {
    _recordLimit?.cancel();
    if (!_isRecording) return;
    final path = await _recorder.stop();
    setState(() {
      _isRecording = false;
      _isTranscribing = true;
    });
    String? error;
    String text = '';
    try {
      final bytes = await File(path!).readAsBytes();
      final res = await http
          .post(Uri.parse('${widget.baseUrl}/api/visitor_voice'), headers: {'Content-Type': 'audio/wav'}, body: bytes)
          .timeout(const Duration(seconds: 50)); // en la charla incluye la respuesta de Gemini
      final data = json.decode(res.body);
      text = (data['transcript'] as String? ?? '').trim();
      if (res.statusCode != 200) error = data['error'] as String? ?? 'Error del servidor (${res.statusCode})';
    } catch (e) {
      error = 'Sin conexión con el robot';
    }
    if (!mounted) return;
    setState(() => _isTranscribing = false);
    if (error != null) {
      _showSnack('🎤 $error');
    } else if (text.isEmpty) {
      _showSnack('🎤 No se entendió, inténtalo de nuevo');
    }
    _fetchState();
  }

  void _showSnack(String text) {
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(text), duration: const Duration(seconds: 3), backgroundColor: const Color(0xFF7C4DFF)));
  }

  @override
  Widget build(BuildContext context) {
    final failed = _failures.contains(_phase);
    final step = _isRecording || _isTranscribing ? 0 : (_stepOf[_phase] ?? -1);
    return Column(
      children: [
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
          color: const Color(0xFF131A29),
          child: Row(
            children: [
              Container(
                width: 36,
                height: 36,
                decoration: BoxDecoration(
                  gradient: const LinearGradient(colors: [_accent, Color(0xFFFF7043)]),
                  shape: BoxShape.circle,
                  boxShadow: [BoxShadow(color: _accent.withOpacity(0.4), blurRadius: 8)],
                ),
                child: const Icon(Icons.person_pin_circle_rounded, color: Colors.black, size: 20),
              ),
              const SizedBox(width: 12),
              const Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Modo visitante', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 14)),
                    Text('Habla como alguien del museo: Pepper te busca, se acerca y charla contigo', style: TextStyle(fontSize: 11, color: Colors.white60)),
                  ],
                ),
              ),
            ],
          ),
        ),

        // Cámara con las detecciones de YOLO (sólo con la pestaña visible)
        Padding(
          padding: const EdgeInsets.fromLTRB(12, 12, 12, 8),
          child: ClipRRect(
            borderRadius: BorderRadius.circular(14),
            child: AspectRatio(
              aspectRatio: 16 / 10,
              child: Container(
                color: Colors.black,
                child: widget.isActive && widget.isConnected
                    ? Mjpeg(
                        isLive: true,
                        stream: '${widget.baseUrl}/stream/yolo',
                        timeout: const Duration(seconds: 3),
                        error: (context, error, stack) => const Center(child: Text('Reconectando cámara...', style: TextStyle(color: Colors.white54))),
                        loading: (context) => const Center(child: CircularProgressIndicator(color: _accent)),
                      )
                    : const Center(child: Text('Sin conexión', style: TextStyle(color: Colors.white54))),
              ),
            ),
          ),
        ),

        // Pasos: escuchar → buscar → acercarse → de frente
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 12),
          child: Row(
            children: [
              for (var i = 0; i < _steps.length; i++) ...[
                Expanded(child: _buildStep(i, step, failed)),
                if (i < _steps.length - 1)
                  Container(width: 6, height: 2, color: (step > i ? _accent : Colors.white12)),
              ],
            ],
          ),
        ),
        if (_statusText.isNotEmpty)
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 8, 16, 4),
            child: Text(_statusText, textAlign: TextAlign.center, style: TextStyle(fontSize: 12, color: failed ? Colors.redAccent : Colors.white70)),
          ),

        // Lo que dijo el visitante y lo que respondió Pepper
        Expanded(
          child: _log.isEmpty
              ? const Center(
                  child: Padding(
                    padding: EdgeInsets.all(24),
                    child: Text('Toca el micrófono y di, por ejemplo,\n"¡Hola Pepper, ven por favor!"', textAlign: TextAlign.center, style: TextStyle(color: Colors.white38, fontSize: 13)),
                  ),
                )
              : ListView.builder(
                  controller: _scrollCtrl,
                  padding: const EdgeInsets.fromLTRB(14, 10, 14, 10),
                  itemCount: _log.length,
                  itemBuilder: (context, i) => _buildLogItem(_log[i] as Map),
                ),
        ),

        Container(
          padding: const EdgeInsets.fromLTRB(16, 10, 16, 14),
          decoration: BoxDecoration(
            color: const Color(0xFF131A29),
            border: Border(top: BorderSide(color: Colors.white.withOpacity(0.08))),
          ),
          child: SafeArea(
            top: false,
            child: Row(
              children: [
                Expanded(
                  child: SizedBox(
                    height: 52,
                    child: ElevatedButton.icon(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: _isRecording ? Colors.redAccent : _accent,
                        foregroundColor: Colors.black,
                        disabledBackgroundColor: Colors.white12,
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(26)),
                      ),
                      onPressed: widget.isConnected && !_isTranscribing ? _toggleRecording : null,
                      icon: _isTranscribing
                          ? const SizedBox(width: 20, height: 20, child: CircularProgressIndicator(strokeWidth: 2, color: Colors.black))
                          : Icon(_isRecording ? Icons.stop_rounded : Icons.mic_rounded),
                      label: Text(
                        _isRecording
                            ? 'Toca para enviar'
                            : _isTranscribing
                            ? (_talking ? 'Pepper piensa...' : 'Transcribiendo...')
                            : _busy
                            ? 'Pepper está en camino...'
                            : _talking
                            ? 'Responder a Pepper'
                            : 'Hablar como visitante',
                        style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 15),
                      ),
                    ),
                  ),
                ),
                // Terminar la charla a mano (si el visitante "se va" sin despedirse)
                if (_talking && !_isRecording && !_isTranscribing) ...[
                  const SizedBox(width: 10),
                  SizedBox(
                    height: 52,
                    child: OutlinedButton(
                      style: OutlinedButton.styleFrom(
                        foregroundColor: Colors.white70,
                        side: const BorderSide(color: Colors.white24),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(26)),
                      ),
                      onPressed: _endTalk,
                      child: const Text('Terminar'),
                    ),
                  ),
                ],
              ],
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildStep(int i, int step, bool failed) {
    final done = step > i || step == _steps.length;
    final current = step == i;
    final color = done ? _accent : current ? Colors.white : Colors.white30;
    return Column(
      children: [
        Container(
          width: 34,
          height: 34,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: done ? _accent.withOpacity(0.18) : Colors.transparent,
            border: Border.all(color: current ? _accent : color.withOpacity(0.6), width: current ? 2 : 1),
          ),
          child: current && step > 0 && step < _steps.length - 1 // en la charla, el ícono sin spinner
              ? const Padding(padding: EdgeInsets.all(8), child: CircularProgressIndicator(strokeWidth: 2, color: _accent))
              : Icon(done ? Icons.check_rounded : _steps[i].$2, size: 18, color: color),
        ),
        const SizedBox(height: 4),
        Text(_steps[i].$1, style: TextStyle(fontSize: 10.5, color: color, fontWeight: current ? FontWeight.bold : FontWeight.normal)),
      ],
    );
  }

  Widget _buildLogItem(Map e) {
    final kind = e['kind'] as String? ?? '';
    final text = e['text'] as String? ?? '';
    if (kind == 'phase') {
      final bad = _failures.contains(e['phase']);
      return Padding(
        padding: const EdgeInsets.symmetric(vertical: 4),
        child: Row(
          children: [
            Icon(bad ? Icons.error_outline_rounded : Icons.radio_button_checked_rounded, size: 12, color: bad ? Colors.redAccent : Colors.white38),
            const SizedBox(width: 8),
            Expanded(child: Text(text, style: TextStyle(fontSize: 11.5, color: bad ? Colors.redAccent : Colors.white54, fontStyle: FontStyle.italic))),
          ],
        ),
      );
    }
    final visitor = kind == 'visitor';
    return Align(
      alignment: visitor ? Alignment.centerRight : Alignment.centerLeft,
      child: Container(
        margin: const EdgeInsets.symmetric(vertical: 4),
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        constraints: const BoxConstraints(maxWidth: 280),
        decoration: BoxDecoration(
          color: visitor ? _accent.withOpacity(0.16) : const Color(0xFF162033),
          borderRadius: BorderRadius.circular(14),
          border: Border.all(color: visitor ? _accent.withOpacity(0.4) : Colors.white10),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(visitor ? '🧑 Visitante' : '🤖 Pepper', style: TextStyle(fontSize: 10, color: visitor ? _accent : const Color(0xFF00E5FF), fontWeight: FontWeight.bold)),
            const SizedBox(height: 2),
            Text(text, style: const TextStyle(fontSize: 13.5)),
          ],
        ),
      ),
    );
  }
}
