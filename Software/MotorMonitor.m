function MotorMonitor()
%  MOTORMONITOR - MATLAB GUI for ESP32-S3 BLDC Motor Controller
% 
% Equivalent to motor_monitor.py (PyQt6 version) CSV format: ID,pvdd,i1,i2,i3,a_raw,b_raw,c_raw,nfault,noctw,ntc,m_ang,red_ang,m_rpm,red_rpm
%% ── Constants ────────────────────────────────────────────
    PLOT_LEN = 500;
%% ── State ────────────────────────────────────────────────
    s.serial_obj   = [];
    s.recording    = false;
    s.rec_fid      = -1;
    s.buf_idx      = 1;
    s.connected    = false;
    s.timer_obj    = [];
    s.raw_buf      = '';
    % Ring buffers
    fields = {'pvdd','i1','i2','i3','m_rpm','red_rpm','ntc','m_ang','a_raw','b_raw','c_raw'};
    for k = 1:numel(fields)
        s.buf.(fields{k}) = zeros(1, PLOT_LEN);
    end
    s.buf_ptr = 1;
    s.x = 1:PLOT_LEN;
%% ── Figure ───────────────────────────────────────────────
    fig = figure('Name','Motor Monitor  //  ESP32-S3 Controller', ...
        'NumberTitle','off', 'Color',[0.055 0.067 0.090], ...
        'Position',[80 60 1280 820], 'CloseRequestFcn',@on_close, ...
        'MenuBar','none','ToolBar','none','Resize','on');
%% ── Toolbar ──────────────────────────────────────────────
    toolbar_h = 52;
    fig_pos = get(fig,'Position');
    W = fig_pos(3); H = fig_pos(4);
    tb = uipanel(fig,'BackgroundColor',[0.031 0.043 0.063], ...
        'BorderType','none','Units','pixels', ...
        'Position',[0 H-toolbar_h W toolbar_h]);
    % Logo
    uicontrol(tb,'Style','text','String','ASTURION  //  Motor Monitor', ...
        'ForegroundColor',[0 0.831 0.667],'BackgroundColor',[0.031 0.043 0.063], ...
        'FontSize',13,'FontWeight','bold','FontName','Courier New', ...
        'Position',[12 12 260 26],'HorizontalAlignment','left');
    % Port label + dropdown
    uicontrol(tb,'Style','text','String','Port:', ...
        'ForegroundColor',[0.784 0.804 0.847],'BackgroundColor',[0.031 0.043 0.063], ...
        'FontSize',10,'Position',[290 16 36 20],'HorizontalAlignment','left');
    s.cb_port = uicontrol(tb,'Style','popupmenu','String',{' '}, ...
        'BackgroundColor',[0.086 0.106 0.153],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Position',[330 14 160 24]);
    % Refresh button
    uicontrol(tb,'Style','pushbutton','String','↺', ...
        'BackgroundColor',[0.102 0.125 0.188],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',12,'Position',[496 14 30 24],'Callback',@refresh_ports);
    % Baud label + dropdown
    uicontrol(tb,'Style','text','String','Baud:', ...
        'ForegroundColor',[0.784 0.804 0.847],'BackgroundColor',[0.031 0.043 0.063], ...
        'FontSize',10,'Position',[534 16 36 20],'HorizontalAlignment','left');
    s.cb_baud = uicontrol(tb,'Style','popupmenu', ...
        'String',{'9600','19200','38400','57600','115200','230400','460800','921600'}, ...
        'BackgroundColor',[0.086 0.106 0.153],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Position',[574 14 90 24],'Value',5);
    % Connect / Disconnect
    s.btn_connect = uicontrol(tb,'Style','pushbutton','String','Connect', ...
        'BackgroundColor',[0.051 0.239 0.180],'ForegroundColor',[0 0.831 0.667], ...
        'FontSize',10,'FontWeight','bold','Position',[672 12 80 28],'Callback',@do_connect);
    s.btn_disconnect = uicontrol(tb,'Style','pushbutton','String','Disconnect', ...
        'BackgroundColor',[0.239 0.071 0.125],'ForegroundColor',[1 0.302 0.427], ...
        'FontSize',10,'FontWeight','bold','Position',[758 12 90 28],'Enable','off','Callback',@do_disconnect);
    % Status label
    s.lbl_conn = uicontrol(tb,'Style','text','String','● OFFLINE', ...
        'ForegroundColor',[1 0.302 0.427],'BackgroundColor',[0.031 0.043 0.063], ...
        'FontSize',11,'FontWeight','bold','Position',[860 14 100 22]);
    % Record button
    s.btn_record = uicontrol(tb,'Style','pushbutton','String','⏺  Save CSV', ...
        'BackgroundColor',[0.239 0.125 0],'ForegroundColor',[1 0.702 0.278], ...
        'FontSize',10,'Position',[1160 12 100 28],'Callback',@toggle_record);
%% ── Tab group ────────────────────────────────────────────
    content_h = H - toolbar_h - 28;  % leave space for status bar
    tg = uitabgroup(fig,'Units','pixels','Position',[0 28 W content_h]);
    tab_dash  = uitab(tg,'Title','  Dashboard  ','BackgroundColor',[0.055 0.067 0.090]);
    tab_ctrl  = uitab(tg,'Title','  Control  ',  'BackgroundColor',[0.055 0.067 0.090]);
    tab_cur   = uitab(tg,'Title','  Currents  ', 'BackgroundColor',[0.055 0.067 0.090]);
    tab_mech  = uitab(tg,'Title','  Mechanical  ','BackgroundColor',[0.055 0.067 0.090]);
    tab_pwr   = uitab(tg,'Title','  Power - NTC  ','BackgroundColor',[0.055 0.067 0.090]);
    tab_con   = uitab(tg,'Title','  Raw Console  ','BackgroundColor',[0.055 0.067 0.090]);
%% ── Status bar ───────────────────────────────────────────
    s.lbl_status = uicontrol(fig,'Style','text', ...
        'String','  No Connection','HorizontalAlignment','left', ...
        'ForegroundColor',[1 0.702 0.278],'BackgroundColor',[0.031 0.043 0.063], ...
        'FontSize',10,'FontName','Courier New','Units','pixels','Position',[0 0 W 24]);
%% ════════════════════════════════════════════════════════
%% TAB 1 — Dashboard
%% ════════════════════════════════════════════════════════
    bg = [0.055 0.067 0.090];
    card_bg = [0.086 0.106 0.153];
    % Metric cards: [name, unit, row, col]
    card_defs = { ...
        'PVDD','V',1,1; 'Current 1','A',1,2; 'Current 2','A',1,3; 'Current 3','A',1,4; ...
        'Motor RPM','rpm',2,1; 'Reducer RPM','rpm',2,2; 'Temperature','°C',2,3; 'State','',2,4};
    card_w = 148; card_h = 80; card_pad = 10;
    for ci = 1:size(card_defs,1)
        nm = card_defs{ci,1}; un = card_defs{ci,2};
        rr = card_defs{ci,3}; cc = card_defs{ci,4};
        xp = card_pad + (cc-1)*(card_w+card_pad);
        yp = content_h - 16 - rr*(card_h+card_pad);
        p = uipanel(tab_dash,'BackgroundColor',card_bg,'BorderType','line', ...
            'HighlightColor',[0.122 0.145 0.204],'Units','pixels','Position',[xp yp card_w card_h]);
        uicontrol(p,'Style','text','String',upper(nm), ...
            'ForegroundColor',[0.478 0.514 0.600],'BackgroundColor',card_bg, ...
            'FontSize',9,'FontWeight','bold','Units','pixels','Position',[6 card_h-22 card_w-12 16]);
        tag = strrep(lower(nm),' ','_');
        s.(['card_' tag '_val']) = uicontrol(p,'Style','text','String','—', ...
            'ForegroundColor',[0 0.831 0.667],'BackgroundColor',card_bg, ...
            'FontSize',20,'FontWeight','bold','Units','pixels','Position',[6 card_h-52 card_w-12 30]);
        uicontrol(p,'Style','text','String',un, ...
            'ForegroundColor',[0.353 0.376 0.439],'BackgroundColor',card_bg, ...
            'FontSize',9,'Units','pixels','Position',[6 4 card_w-12 16]);
    end
    % Digital status labels
    dig_y = content_h - 16 - 2*(card_h+card_pad) - 36;
    dig_labels = {'ID: —','NFault: —','OC Warn: —','Motor Angle: —°','Reducer Angle: —°'};
    dig_tags   = {'lbl_id','lbl_nfault','lbl_noctw','lbl_ang','lbl_redang'};
    for di = 1:5
        s.(dig_tags{di}) = uicontrol(tab_dash,'Style','text','String',dig_labels{di}, ...
            'ForegroundColor',[0.784 0.804 0.847],'BackgroundColor',bg, ...
            'FontSize',10,'FontName','Courier New', ...
            'Units','pixels','Position',[card_pad + (di-1)*220 dig_y 210 22],'HorizontalAlignment','left');
    end
    % RPM quick-view plot
    ax_drpm = axes('Parent',tab_dash,'Units','pixels', ...
        'Position',[card_pad dig_y-200 W-2*card_pad 175], ...
        'Color',[0.055 0.067 0.090],'XColor',[0.353 0.376 0.439],'YColor',[0.353 0.376 0.439], ...
        'GridColor',[0.2 0.2 0.2],'GridAlpha',0.3,'XGrid','on','YGrid','on','FontSize',9);
    title(ax_drpm,'Motor RPM (quick view)','Color',[0.478 0.514 0.600],'FontSize',10);
    ylabel(ax_drpm,'RPM','Color',[0.353 0.376 0.439]);
    s.crv_dash_rpm(1) = line(ax_drpm, s.x, zeros(1,PLOT_LEN), 'Color',[0 0.831 0.667],'LineWidth',1.5,'DisplayName','Motor RPM');
    s.crv_dash_rpm(2) = line(ax_drpm, s.x, zeros(1,PLOT_LEN), 'Color',[0.306 0.620 1.000],'LineWidth',1.5,'DisplayName','Reducer RPM');
    legend(ax_drpm,'TextColor',[0.784 0.804 0.847],'Color',[0.086 0.106 0.153]);
%% ════════════════════════════════════════════════════════
%% TAB 2 — Control
%% ════════════════════════════════════════════════════════
    ctrl_lw = 360; ctrl_x = 24;
    % Motor state group
    uipanel(tab_ctrl,'Title','Motor State','BackgroundColor',bg, ...
        'ForegroundColor',[0.478 0.514 0.600],'Units','pixels', ...
        'Position',[ctrl_x content_h-180 ctrl_lw 160],'FontSize',10);
    state_names = {'State:','Motor RPM:','PVDD:','NFAULT:','Motor ID:'};
    state_tags  = {'lbl_ctrl_state','lbl_ctrl_rpm','lbl_ctrl_pvdd','lbl_ctrl_nfault','lbl_ctrl_id'};
    for si = 1:5
        row = ceil(si/2); col = mod(si-1,2);
        uicontrol(tab_ctrl,'Style','text','String',state_names{si}, ...
            'ForegroundColor',[0.478 0.514 0.600],'BackgroundColor',bg, ...
            'FontSize',10,'Units','pixels', ...
            'Position',[ctrl_x+10+col*170 content_h-180+120-row*36 80 20],'HorizontalAlignment','left');
        s.(state_tags{si}) = uicontrol(tab_ctrl,'Style','text','String','—', ...
            'ForegroundColor',[0.784 0.804 0.847],'BackgroundColor',bg, ...
            'FontSize',10,'FontWeight','bold','Units','pixels', ...
            'Position',[ctrl_x+90+col*170 content_h-180+120-row*36 80 20],'HorizontalAlignment','left');
    end
    % Commands group
    uipanel(tab_ctrl,'Title','Commands (sends via UART)','BackgroundColor',bg, ...
        'ForegroundColor',[0.478 0.514 0.600],'Units','pixels', ...
        'Position',[ctrl_x content_h-380 ctrl_lw 185],'FontSize',10);
    btn_defs = { ...
        '⏸  IDLE',  'CMD:IDLE',  ctrl_x+10,           content_h-260; ...
        '⚙  Align', 'CMD:ALIGN', ctrl_x+10+82,        content_h-260; ...
        '⏹  Brake', 'CMD:BRAKE', ctrl_x+10+82*3,      content_h-260};
    for bi = 1:size(btn_defs,1)
        uicontrol(tab_ctrl,'Style','pushbutton','String',btn_defs{bi,1}, ...
            'BackgroundColor',[0.102 0.125 0.188],'ForegroundColor',[0.784 0.804 0.847], ...
            'FontSize',10,'Units','pixels','Position',[btn_defs{bi,3} btn_defs{bi,4} 78 30], ...
            'Callback',{@send_cmd, [btn_defs{bi,2} char(10)]});
    end
    % RUN button + freq input
    s.run_freq = uicontrol(tab_ctrl,'Style','edit','String','30', ...
        'BackgroundColor',[0.086 0.106 0.153],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Units','pixels','Position',[ctrl_x+10+82*2+52 content_h-260 26 30]);
    uicontrol(tab_ctrl,'Style','pushbutton','String','▶  RUN', ...
        'BackgroundColor',[0.102 0.125 0.188],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Units','pixels','Position',[ctrl_x+10+82*2 content_h-260 52 30], ...
        'Callback',@handle_run);
    % Feedback method
    uicontrol(tab_ctrl,'Style','text','String','Feedback Method:', ...
        'ForegroundColor',[0.478 0.514 0.600],'BackgroundColor',bg, ...
        'FontSize',10,'Units','pixels','Position',[ctrl_x+10 content_h-308 130 20],'HorizontalAlignment','left');
    s.cb_fbk = uicontrol(tab_ctrl,'Style','popupmenu', ...
        'String',{'MAG_ENCODER','HALL_SENSOR','FOC_ANGLE'}, ...
        'BackgroundColor',[0.086 0.106 0.153],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Units','pixels','Position',[ctrl_x+140 content_h-310 200 26], ...
        'Callback',@send_fbk);
    % STOP button
    uicontrol(tab_ctrl,'Style','pushbutton','String','🛑  STOP', ...
        'BackgroundColor',[0.239 0.051 0.051],'ForegroundColor',[1 0.302 0.427], ...
        'FontSize',12,'FontWeight','bold','Units','pixels', ...
        'Position',[ctrl_x content_h-380+10 ctrl_lw-20 34],'Callback',{@send_cmd,['CMD:STOP' char(10)]});
    % Free text group
    uipanel(tab_ctrl,'Title','Send Free Text','BackgroundColor',bg, ...
        'ForegroundColor',[0.478 0.514 0.600],'Units','pixels', ...
        'Position',[ctrl_x content_h-460 ctrl_lw 70],'FontSize',10);
    s.tx_edit = uicontrol(tab_ctrl,'Style','edit','String','', ...
        'BackgroundColor',[0.086 0.106 0.153],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Units','pixels','Position',[ctrl_x+10 content_h-448 ctrl_lw-100 28], ...
        'Callback',@send_manual);
    uicontrol(tab_ctrl,'Style','pushbutton','String','Send', ...
        'BackgroundColor',[0.102 0.125 0.188],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Units','pixels','Position',[ctrl_x+ctrl_lw-82 content_h-448 62 28], ...
        'Callback',@send_manual);
%% ════════════════════════════════════════════════════════
%% TAB 3 — Currents
%% ════════════════════════════════════════════════════════
    ax_cur = axes('Parent',tab_cur,'Units','normalized','Position',[0.06 0.55 0.92 0.40], ...
        'Color',bg,'XColor',[0.353 0.376 0.439],'YColor',[0.353 0.376 0.439], ...
        'GridColor',[0.2 0.2 0.2],'GridAlpha',0.3,'XGrid','on','YGrid','on');
    title(ax_cur,'Phase Currents','Color',[0.478 0.514 0.600],'FontSize',10);
    ylabel(ax_cur,'Amps','Color',[0.353 0.376 0.439]);
    s.crv_cur(1) = line(ax_cur,s.x,zeros(1,PLOT_LEN),'Color',[1 0.302 0.427],'LineWidth',1.5,'DisplayName','I1');
    s.crv_cur(2) = line(ax_cur,s.x,zeros(1,PLOT_LEN),'Color',[1 0.702 0.278],'LineWidth',1.5,'DisplayName','I2');
    s.crv_cur(3) = line(ax_cur,s.x,zeros(1,PLOT_LEN),'Color',[0.306 0.620 1.000],'LineWidth',1.5,'DisplayName','I3');
    legend(ax_cur,'TextColor',[0.784 0.804 0.847],'Color',[0.086 0.106 0.153]);
    ax_raw = axes('Parent',tab_cur,'Units','normalized','Position',[0.06 0.07 0.92 0.40], ...
        'Color',bg,'XColor',[0.353 0.376 0.439],'YColor',[0.353 0.376 0.439], ...
        'GridColor',[0.2 0.2 0.2],'GridAlpha',0.3,'XGrid','on','YGrid','on');
    title(ax_raw,'ADC phases (A, B, C)','Color',[0.478 0.514 0.600],'FontSize',10);
    ylabel(ax_raw,'Raw','Color',[0.353 0.376 0.439]);
    s.crv_raw(1) = line(ax_raw,s.x,zeros(1,PLOT_LEN),'Color',[1 0.302 0.427],'LineWidth',1.5,'DisplayName','A_raw');
    s.crv_raw(2) = line(ax_raw,s.x,zeros(1,PLOT_LEN),'Color',[0.306 0.620 1.000],'LineWidth',1.5,'DisplayName','B_raw');
    s.crv_raw(3) = line(ax_raw,s.x,zeros(1,PLOT_LEN),'Color',[0 0.831 0.667],'LineWidth',1.5,'DisplayName','C_raw');
    legend(ax_raw,'TextColor',[0.784 0.804 0.847],'Color',[0.086 0.106 0.153]);
%% ════════════════════════════════════════════════════════
%% TAB 4 — Mechanical
%% ════════════════════════════════════════════════════════
    ax_rpm = axes('Parent',tab_mech,'Units','normalized','Position',[0.06 0.55 0.92 0.40], ...
        'Color',bg,'XColor',[0.353 0.376 0.439],'YColor',[0.353 0.376 0.439], ...
        'GridColor',[0.2 0.2 0.2],'GridAlpha',0.3,'XGrid','on','YGrid','on');
    title(ax_rpm,'Angular Velocity','Color',[0.478 0.514 0.600],'FontSize',10);
    ylabel(ax_rpm,'RPM','Color',[0.353 0.376 0.439]);
    s.crv_rpm(1) = line(ax_rpm,s.x,zeros(1,PLOT_LEN),'Color',[0 0.831 0.667],'LineWidth',1.5,'DisplayName','Motor');
    s.crv_rpm(2) = line(ax_rpm,s.x,zeros(1,PLOT_LEN),'Color',[0.306 0.620 1.000],'LineWidth',1.5,'DisplayName','Reducer');
    legend(ax_rpm,'TextColor',[0.784 0.804 0.847],'Color',[0.086 0.106 0.153]);
    ax_ang = axes('Parent',tab_mech,'Units','normalized','Position',[0.06 0.07 0.92 0.40], ...
        'Color',bg,'XColor',[0.353 0.376 0.439],'YColor',[0.353 0.376 0.439], ...
        'GridColor',[0.2 0.2 0.2],'GridAlpha',0.3,'XGrid','on','YGrid','on');
    title(ax_ang,'Angle','Color',[0.478 0.514 0.600],'FontSize',10);
    ylabel(ax_ang,'Degrees','Color',[0.353 0.376 0.439]);
    s.crv_ang = line(ax_ang,s.x,zeros(1,PLOT_LEN),'Color',[0.780 0.490 1.000],'LineWidth',1.5,'DisplayName','Motor angle');
    legend(ax_ang,'TextColor',[0.784 0.804 0.847],'Color',[0.086 0.106 0.153]);
%% ════════════════════════════════════════════════════════
%% TAB 5 — Power / NTC
%% ════════════════════════════════════════════════════════
    ax_pvdd = axes('Parent',tab_pwr,'Units','normalized','Position',[0.06 0.55 0.92 0.40], ...
        'Color',bg,'XColor',[0.353 0.376 0.439],'YColor',[0.353 0.376 0.439], ...
        'GridColor',[0.2 0.2 0.2],'GridAlpha',0.3,'XGrid','on','YGrid','on');
    title(ax_pvdd,'Power Supply','Color',[0.478 0.514 0.600],'FontSize',10);
    ylabel(ax_pvdd,'Volts','Color',[0.353 0.376 0.439]);
    s.crv_pvdd = line(ax_pvdd,s.x,zeros(1,PLOT_LEN),'Color',[1 0.702 0.278],'LineWidth',1.5,'DisplayName','PVDD');
    yline(ax_pvdd,16,'--','Color',[1 0.302 0.427],'LineWidth',1,'Label','Max 16V');
    legend(ax_pvdd,'TextColor',[0.784 0.804 0.847],'Color',[0.086 0.106 0.153]);
    ax_ntc = axes('Parent',tab_pwr,'Units','normalized','Position',[0.06 0.07 0.92 0.40], ...
        'Color',bg,'XColor',[0.353 0.376 0.439],'YColor',[0.353 0.376 0.439], ...
        'GridColor',[0.2 0.2 0.2],'GridAlpha',0.3,'XGrid','on','YGrid','on');
    title(ax_ntc,'Temperature NTC','Color',[0.478 0.514 0.600],'FontSize',10);
    ylabel(ax_ntc,'°C','Color',[0.353 0.376 0.439]);
    s.crv_ntc = line(ax_ntc,s.x,zeros(1,PLOT_LEN),'Color',[1 0.420 0.420],'LineWidth',1.5,'DisplayName','NTC');
    yline(ax_ntc,60,'--','Color',[1 0.702 0.278],'LineWidth',1,'Label','Warn 60°C');
    legend(ax_ntc,'TextColor',[0.784 0.804 0.847],'Color',[0.086 0.106 0.153]);
%% ════════════════════════════════════════════════════════
%% TAB 6 — Raw Console
%% ════════════════════════════════════════════════════════
    s.console = uicontrol(tab_con,'Style','listbox','String',{}, ...
        'BackgroundColor',[0.031 0.043 0.063],'ForegroundColor',[0.478 1 0.600], ...
        'FontSize',10,'FontName','Courier New','Units','normalized', ...
        'Position',[0.01 0.08 0.98 0.87],'HorizontalAlignment','left');
    uicontrol(tab_con,'Style','pushbutton','String','Clear Console', ...
        'BackgroundColor',[0.102 0.125 0.188],'ForegroundColor',[0.784 0.804 0.847], ...
        'FontSize',10,'Units','normalized','Position',[0.01 0.01 0.15 0.05], ...
        'Callback',@(~,~) set(s.console,'String',{}));
%% ── Init port list ───────────────────────────────────────
    refresh_ports();
%% ════════════════════════════════════════════════════════
%% Nested callback functions
%% ════════════════════════════════════════════════════════
    function refresh_ports(~,~)
        try
            ports = serialportlist("available");
        catch
            ports = {};
        end
        if isempty(ports)
            set(s.cb_port,'String',{'(No ports detected)'},'Value',1);
        else
            set(s.cb_port,'String',cellstr(ports),'Value',1);
        end
        set_status('Ports refreshed','warn');
    end
    function do_connect(~,~)
        ports = get(s.cb_port,'String');
        idx   = get(s.cb_port,'Value');
        port  = ports{idx};
        if contains(port,'No ports')
            set_status('Select a valid port','err'); return;
        end
        bauds = {'9600','19200','38400','57600','115200','230400','460800','921600'};
        baud  = str2double(bauds{get(s.cb_baud,'Value')});
        try
            s.serial_obj = serialport(port, baud, 'Timeout', 0.1);
            configureTerminator(s.serial_obj, 'LF');
            s.connected = true;
        catch e
            set_status(['Cannot open ' port ': ' e.message],'err'); return;
        end
        % Timer-based polling at 50 Hz
        s.timer_obj = timer('ExecutionMode','fixedRate','Period',0.02, ...
            'TimerFcn',@poll_serial,'ErrorFcn',@timer_error);
        start(s.timer_obj);
        set(s.btn_connect,'Enable','off');
        set(s.btn_disconnect,'Enable','on');
        set(s.lbl_conn,'String','● ONLINE','ForegroundColor',[0 0.831 0.667]);
        set_status(['Connected to ' port ' @ ' num2str(baud) ' baud'],'ok');
    end
    function do_disconnect(~,~)
        if ~isempty(s.timer_obj) && isvalid(s.timer_obj)
            stop(s.timer_obj); delete(s.timer_obj); s.timer_obj = [];
        end
        if ~isempty(s.serial_obj) && isvalid(s.serial_obj)
            delete(s.serial_obj); s.serial_obj = [];
        end
        s.connected = false;
        set(s.btn_connect,'Enable','on');
        set(s.btn_disconnect,'Enable','off');
        set(s.lbl_conn,'String','● OFFLINE','ForegroundColor',[1 0.302 0.427]);
        set_status('Disconnected','warn');
        if s.recording, toggle_record(); end
    end
    function poll_serial(~,~)
        if isempty(s.serial_obj) || ~isvalid(s.serial_obj), return; end
        try
            while s.serial_obj.NumBytesAvailable > 0
                chunk = read(s.serial_obj, s.serial_obj.NumBytesAvailable, 'char');
                s.raw_buf = [s.raw_buf chunk];
            end
        catch
            return;
        end
        % Process complete lines
        while true
            nl = strfind(s.raw_buf, newline);
            if isempty(nl), break; end
            line = s.raw_buf(1:nl(1)-1);
            s.raw_buf = s.raw_buf(nl(1)+1:end);
            line = strtrim(line);
            if isempty(line), continue; end
            % Append to console
            cur = get(s.console,'String');
            if ischar(cur), cur = {cur}; end
            cur{end+1} = line;
            if numel(cur) > 500, cur = cur(end-499:end); end
            set(s.console,'String',cur,'Value',numel(cur));
            % Try parse CSV (15 fields)
            parts = strsplit(line, ',');
            if numel(parts) == 15
                vals = str2double(parts);
                if ~any(isnan(vals))
                    on_data(vals);
                end
            end
        end
    end
    function timer_error(~,~)
        set_status('Timer error — disconnecting','err');
        do_disconnect();
    end
    function on_data(v)
        % v: [ID pvdd i1 i2 i3 a_raw b_raw c_raw nfault noctw ntc m_ang red_ang m_rpm red_rpm]
        d.pvdd    = v(2);  d.i1     = v(3);  d.i2      = v(4);  d.i3    = v(5);
        d.a_raw   = v(6);  d.b_raw  = v(7);  d.c_raw   = v(8);
        d.nfault  = v(9);  d.noctw  = v(10); d.ntc     = v(11);
        d.m_ang   = v(12); d.red_ang= v(13); d.m_rpm   = v(14); d.red_rpm = v(15);
        d.ID      = v(1);
        % Update ring buffers
        p = s.buf_ptr;
        s.buf.pvdd(p)   = d.pvdd;   s.buf.i1(p)    = d.i1;
        s.buf.i2(p)     = d.i2;     s.buf.i3(p)    = d.i3;
        s.buf.a_raw(p)  = d.a_raw;  s.buf.b_raw(p) = d.b_raw;
        s.buf.c_raw(p)  = d.c_raw;  s.buf.m_rpm(p) = d.m_rpm;
        s.buf.red_rpm(p)= d.red_rpm; s.buf.ntc(p)  = d.ntc;
        s.buf.m_ang(p)  = d.m_ang;
        s.buf_ptr = mod(p, PLOT_LEN) + 1;
        % Ordered data for plotting
        ord = [p+1:PLOT_LEN 1:p];
        % Update metric cards
        update_card('pvdd', d.pvdd, 15, 18, '%.2f');
        update_card('current_1', abs(d.i1), 20, 35, '%.2f');
        update_card('current_2', abs(d.i2), 20, 35, '%.2f');
        update_card('current_3', abs(d.i3), 20, 35, '%.2f');
        update_card('motor_rpm', d.m_rpm, inf, inf, '%.1f');
        update_card('reducer_rpm', d.red_rpm, inf, inf, '%.2f');
        update_card('temperature', d.ntc, 60, 80, '%.1f');
        set(s.card_state_val,'String','RUNNING');
        % Digital status
        nf_ok = d.nfault == 1; oc_ok = d.noctw == 1;
        set(s.lbl_id,     'String', sprintf('ID: %d', d.ID));
        set(s.lbl_nfault, 'String', sprintf('NFault: %s', ternary(nf_ok,'OK','FAULT')), ...
            'ForegroundColor', ternary(nf_ok,[0 0.831 0.667],[1 0.302 0.427]));
        set(s.lbl_noctw,  'String', sprintf('OC Warn: %s', ternary(oc_ok,'OK','WARN')), ...
            'ForegroundColor', ternary(oc_ok,[0 0.831 0.667],[1 0.702 0.278]));
        set(s.lbl_ang,    'String', sprintf('Motor Angle: %.2f°', d.m_ang));
        set(s.lbl_redang, 'String', sprintf('Reducer Angle: %.2f°', d.red_ang));
        % Control tab
        set(s.lbl_ctrl_rpm,    'String', sprintf('%.1f rpm', d.m_rpm));
        set(s.lbl_ctrl_pvdd,   'String', sprintf('%.2f V',   d.pvdd));
        set(s.lbl_ctrl_nfault, 'String', ternary(nf_ok,'OK','FAULT'), ...
            'ForegroundColor', ternary(nf_ok,[0 0.831 0.667],[1 0.302 0.427]));
        set(s.lbl_ctrl_id,     'String', num2str(d.ID));
        % Update plots
        set(s.crv_dash_rpm(1), 'YData', s.buf.m_rpm(ord));
        set(s.crv_dash_rpm(2), 'YData', s.buf.red_rpm(ord));
        set(s.crv_cur(1), 'YData', s.buf.i1(ord));
        set(s.crv_cur(2), 'YData', s.buf.i2(ord));
        set(s.crv_cur(3), 'YData', s.buf.i3(ord));
        set(s.crv_raw(1), 'YData', s.buf.a_raw(ord));
        set(s.crv_raw(2), 'YData', s.buf.b_raw(ord));
        set(s.crv_raw(3), 'YData', s.buf.c_raw(ord));
        set(s.crv_rpm(1), 'YData', s.buf.m_rpm(ord));
        set(s.crv_rpm(2), 'YData', s.buf.red_rpm(ord));
        set(s.crv_ang,    'YData', s.buf.m_ang(ord));
        set(s.crv_pvdd,   'YData', s.buf.pvdd(ord));
        set(s.crv_ntc,    'YData', s.buf.ntc(ord));
        % CSV recording
        if s.recording && s.rec_fid ~= -1
            fprintf(s.rec_fid, '%s,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n', ...
                datestr(now,'yyyy-mm-ddTHH:MM:SS'), ...
                d.pvdd,d.i1,d.i2,d.i3,d.a_raw,d.b_raw,d.c_raw, ...
                d.nfault,d.noctw,d.ntc,d.m_ang,d.red_ang,d.m_rpm,d.red_rpm);
        end
    end
    function update_card(tag, val, warn, danger, fmt)
        h = s.(['card_' tag '_val']);
        set(h,'String', sprintf(fmt, val));
        if val >= danger
            set(h,'ForegroundColor',[1 0.302 0.427]);
        elseif val >= warn
            set(h,'ForegroundColor',[1 0.702 0.278]);
        else
            set(h,'ForegroundColor',[0 0.831 0.667]);
        end
    end
    function send_cmd(~,~,cmd)
        write_serial(cmd);
    end
    function handle_run(~,~)
        freq = strtrim(get(s.run_freq,'String'));
        if isempty(freq)
            write_serial(['CMD:RUN' newline]);
        else
            write_serial(['CMD:RUN:' freq newline]);
        end
    end
    function send_fbk(~,~)
        opts = {'MAG_ENCODER','HALL_SENSOR','FOC_ANGLE'};
        sel  = opts{get(s.cb_fbk,'Value')};
        write_serial(['CMD:FBK:' sel newline]);
    end
    function send_manual(~,~)
        txt = strtrim(get(s.tx_edit,'String'));
        if ~isempty(txt)
            write_serial([txt newline]);
            set(s.tx_edit,'String','');
        end
    end
    function write_serial(cmd)
        if s.connected && ~isempty(s.serial_obj) && isvalid(s.serial_obj)
            try
                write(s.serial_obj, cmd, 'char');
                set_status(['TX → ' strtrim(cmd)],'ok');
            catch e
                set_status(['Write failed: ' e.message],'err');
            end
        else
            set_status('No active connection','warn');
        end
    end
    function toggle_record(~,~)
        if ~s.recording
            [fname, fpath] = uiputfile('*.csv','Save CSV', ...
                ['telem_' datestr(now,'yyyymmdd_HHMMSS') '.csv']);
            if isequal(fname,0), return; end
            fullpath = fullfile(fpath,fname);
            s.rec_fid = fopen(fullpath,'w');
            fprintf(s.rec_fid, 'timestamp,pvdd,i1,i2,i3,a_raw,b_raw,c_raw,nfault,noctw,ntc,m_ang,red_ang,m_rpm,red_rpm\n');
            s.recording = true;
            set(s.btn_record,'String','⏹  Stop Recording');
            set_status(['Recording → ' fullpath],'ok');
        else
            s.recording = false;
            if s.rec_fid ~= -1, fclose(s.rec_fid); s.rec_fid = -1; end
            set(s.btn_record,'String','⏺  Save CSV');
            set_status('Recording stopped','warn');
        end
    end
    function set_status(msg, level)
        colors = struct('ok',[0 0.831 0.667],'warn',[1 0.702 0.278],'err',[1 0.302 0.427]);
        if isfield(colors,level), col = colors.(level); else, col = [0.784 0.804 0.847]; end
        set(s.lbl_status,'String',['  ' msg],'ForegroundColor',col);
    end
    function on_close(~,~)
        do_disconnect();
        delete(fig);
    end
end
%% ── Helpers ──────────────────────────────────────────────────
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end